/*
 * modbus.cpp
 *
 *  Created on: Nov 27, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */
#include <type_traits>
#include <cassert>
#include <ostream>
#include "modbus.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
#include "uart.h"
#include "gpio.h"

namespace ModBus {



#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

#define lowByte(w) ((w) & 0xff)
#define highByte(w) ((w) >> 8)

template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
    return static_cast<typename std::underlying_type<E>::type>(e);
}

volatile Modbus *mHandlers[MAX_M_HANDLERS];


void MasterTask(void *argument);
void SlaveTask(void *argument);
//Semaphore to access the Modbus Data
osSemaphoreDef(ModBusSp);

volatile uint8_t numberHandlers = 0;


Register::Register():
	acces(Register::Access::RW),
	mIdx(Register::Index::END),
	mName(""),
	mValues(),
	mGetter(defaultGetter),
	mSetter(defaultSetter),
	mOnChanged(defaultOnChanged),
	mOnAccessError(defaultOnAccessError)
{
	DBG("Register: %i created empty\n", mIdx);
}

Register::Register(Register::Index idx, const char* name, Register::ValuesType values,
		Register::Access acces,
		Register::GetterType getter,
		Register::SetterType setter,
		Register::OnChanged changed,
		Register::OnAccessError error):

	acces(acces),
	mIdx(idx),
	mName(name),
	mValues(values),
	mGetter(getter),
	mSetter(setter),
	mOnChanged(changed),
	mOnAccessError(error)
{
	DBG("Register: %i: %s created\n", mIdx, mName);
}

Register::operator const uint16_t ()
{
	static uint16_t fake = 0;
	if(acces != Access::WO) {
		if(mGetter)
			return mGetter(mValues);
		else
			return defaultGetter(mValues);
	}
	else {
		if(mOnAccessError)
			mOnAccessError(this);
		else
			defaultOnAccessError(this);
	}
	return fake;
}

const uint16_t& Register::operator = (const uint16_t& value)
{
	if(acces != Access::RO) {
		if(mSetter)
			mSetter(mValues, value);
		else
			defaultSetter(mValues, value);
		if(mOnChanged)
			mOnChanged(this);
		else
			defaultOnChanged(this);
	}else{
		if(mOnAccessError)
			mOnAccessError(this);
		else
			defaultOnAccessError(this);
	}
	return value;
}


const uint16_t& Register::SetValue(const uint16_t& value)
{
	defaultSetter(mValues, value);
	return value;
}
const uint16_t& Register::operator &= (const uint16_t& value)
{
	return (*this &= value) ;
}
const uint16_t& Register::operator |= (const uint16_t& value)
{
	return (*this |= value) ;
}

template<typename T>
std::ostream& operator<<(typename std::enable_if<std::is_enum<T>::value, std::ostream>::type& stream, const T& e)
{
    return stream << static_cast<typename std::underlying_type<T>::type>(e);
}

template<typename T>
std::ostream& operator<<(std::ostream& str, const ModBus::Register::RefValue<T>& reg)
{

	return str << std::endl;
}


std::ostream& operator<<(std::ostream& str, const ModBus::Register& reg)
{
	str << "reg: " << reg.mIdx << " acc: "  << reg.acces << " val: " << (uint16_t)reg.mValues.size();
	for(const auto& v : reg.mValues)
	{
		str << "R:"<< streamer(v) << " ";
	}
	return str << std::endl;
}

/* Ring Buffer functions */
// This function must be called only after disabling USART RX interrupt or inside of the RX interrupt

template<typename T, uint8_t S>
uint8_t RingBuffer<T,S>::GetAllBytes(uint8_t *buffer)
{
	return GetNBytes(buffer, mAvailable);
}

// This function must be called only after disabling USART RX interrupt
template<typename T , uint8_t S >
uint8_t RingBuffer<T,S>::GetNBytes(uint8_t *buffer, uint8_t number)
{
	uint8_t counter;
	if(mAvailable == 0  || number == 0 ) return 0;
	if(number > S) return 0;

	for(counter = 0; counter < number && counter< mAvailable ; counter++)
	{
		buffer[counter] = mBuffer[mBegin];
		mBegin = (mBegin + 1) % S;
	}
	mAvailable = mAvailable - counter;
	mOverflow = false;
	Clear();

	return counter;
}
template<typename T, uint8_t S>
uint8_t RingBuffer<T,S>::CountBytes() const
{
return mAvailable;
}

template<typename T, uint8_t S>
void RingBuffer<T,S>::Clear()
{
	mBegin = 0;
	mEnd = 0;
	mAvailable = 0;
	mOverflow = false;
}

/* End of Ring Buffer functions */


/*-------------------------------------------------------------------------------------------------------------------*/

const uint8_t fctsupported[] =
{
	to_underlying(FunctionCode::READ_COILS),
	to_underlying(FunctionCode::READ_DISCRETE_INPUT),
	to_underlying(FunctionCode::READ_REGISTERS),
	to_underlying(FunctionCode::READ_INPUT_REGISTER),
	to_underlying(FunctionCode::WRITE_COIL),
	to_underlying(FunctionCode::WRITE_REGISTER),
	to_underlying(FunctionCode::WRITE_MULTIPLE_COILS),
	to_underlying(FunctionCode::WRITE_MULTIPLE_REGISTERS)
};

const osThreadAttr_t thread_attr = {
  .name = "Modbus",
  .stack_size = 512                            // Create the thread stack with a size of 1024 bytes
};
Modbus::Modbus(Uart_t *uart, Gpio_t *dePin, ModBusType type, const uint8_t id, Registers &regs):
		mType(type),
		mUart(uart),
		mId(1, 247, id, NvVar::MODBUS_SLAVE_ID), //0 - only for master
		mBaudRate(115200/2400, 115200/115200, 115200/9600, NvVar::MODBUS_SPEED),
		mWordLen(UART_8_BIT,UART_9_BIT,UART_8_BIT, NvVar::MODBUS_BITS),
		mStopBits(UART_1_STOP_BIT,UART_1_5_STOP_BIT,UART_1_STOP_BIT, NvVar::MODBUS_STOP_BITS),
		mParity(NO_PARITY, ODD_PARITY, NO_PARITY, NvVar::MODBUS_PARITY),
		mDePin(dePin),
		mLastError(),
		mBuffer(),
		mLastRec(),
		mRegs(regs),
		mInCnt(),
		mOutCnt(),
		mErrCnt(),
		mTimeOut(),
		mDataRX(),
		mState(),
		mBufferRX()
{

	if(mType ==ModBusType::Slave) {
		mRegs.emplace(MODBUS_REGISTER(Index::IDENT, mId, Register::Access::RW,
		[&](const Register::ValuesType &nvp)->uint16_t
		{
			return std::get<Register::nv8_ref>(nvp[0]).get();
		},
		[&](Register::ValuesType &nvp, const uint16_t value)->uint16_t {
			std::get<Register::nv8_ref>(nvp[0]).get() = value;
			return value;
		}));

		mRegs.emplace(MODBUS_REGISTER(Index::BAUD_RATE_AND_WORD_LEN, {mBaudRate COMMA mWordLen}, Register::Access::RW,

		[&](const Register::ValuesType &nvp)->uint16_t
		{
			uint16_t result = std::get<Register::nv8_ref>(nvp[0]).get() << 8 |
						      (std::get<Register::nv8_ref>(nvp[1]).get() & 0xff);
			return result;
		},
		[&](Register::ValuesType &nvp, const uint16_t v)->uint16_t
		{

			std::get<Register::nv8_ref>(nvp[0]).get() = ( v >> 8 ) & 0xff;
			std::get<Register::nv8_ref>(nvp[1]).get() = ( v >> 0 ) & 0xff;
			SetLine();
			return v;
		}));

		mRegs.emplace(MODBUS_REGISTER(Index::STOP_BITS_AND_PARITY, {mStopBits COMMA mParity}, Register::Access::RW,

		[&](const Register::ValuesType &nvp)->uint16_t
		{
			uint16_t result = std::get<Register::nv8_ref>(nvp[0]).get() << 8 |
							 (std::get<Register::nv8_ref>(nvp[1]).get() & 0xff);

			return result;
		},
		[&](Register::ValuesType &nvp, const uint16_t v)->uint16_t
		{
			std::get<Register::nv8_ref>(nvp[0]).get() = ( v >> 8 ) & 0xff;
			std::get<Register::nv8_ref>(nvp[1]).get() = ( v >> 0 ) & 0xff;
			SetLine();
			return v;
		}));
	}
    assert(numberHandlers < MAX_M_HANDLERS);
  	  //Initialize the ring buffer

	 mBufferRX.Clear();

	 mTaskHandle =  osThreadNew((mType == ModBusType::Slave)? SlaveTask : MasterTask, this, &thread_attr);
	 if (mType == ModBusType::Master)
	 {
		 mTimerTimeout = osTimerNew(vTimerCallbackTimeout,  osTimerOnce , &mTimerTimeout, nullptr);
		 if(mTimerTimeout == nullptr)
		 {
			  while(1); //error creating timer, check heap and stack size
		 }

		 mQueueMessageHandle = osMessageQueueNew(MAX_M_HANDLERS, sizeof(Query_t), nullptr);

		 if(mQueueMessageHandle == nullptr)
		 {
			  while(1); //error creating queue for telegrams, check heap and stack size
		 }

	 }
	 else if (mType != ModBusType::Slave)
	 {
	  while(1); //Error Modbus type not supported choose a valid Type
	 }

	 if(mTaskHandle == nullptr)
	 {
	  while(1); //Error creating Modbus task, check heap and stack size
	 }
	 mTimerT35 = osTimerNew(vTimerCallbackT35,  osTimerPeriodic , &mTimerT35, nullptr);

	 mSpHandle = osSemaphoreNew(1, 0, nullptr);

	 mUart->IrqNotify = ModBus_IrqNotify;

	 mHandlers[numberHandlers++] = this;
}

void Modbus::SetLine(const uint32_t baudrate, const WordLength_t wordLength, const StopBits_t stopBits, const Parity_t parity)
{
	UartConfig( mUart, RX_TX, SYNC, baudrate, wordLength, stopBits, parity, NO_FLOW_CTRL );
}

void Modbus::Start( )
{

	if (mDePin != nullptr )
    {
       // return RS485 transceiver to receive mode
		GpioWrite(mDePin, 0);
    }

    if (mType == ModBusType::Slave  &&  mRegs.empty())
    {
      assert(0);
    }

    //check that port is initialized

    assert(UartWaitReady(mUart, TIMEOUT_MODBUS));

    SetLine(115200/mBaudRate, static_cast<WordLength_t>((uint8_t)mWordLen),static_cast<StopBits_t>((uint8_t)mStopBits), static_cast<Parity_t>((uint8_t)mParity));

    assert((mId ==0 && mType == ModBusType::Master ) || (mId !=0 && mType == ModBusType::Slave ));

    mLastRec = mBufferSize = 0;
    mInCnt = mOutCnt = mErrCnt = 0;
}


void Modbus::vTimerCallbackT35(void* arg)
{
	osTimerId *tim = (osTimerId*)arg;
	int i;
	for(i = 0; i < numberHandlers; i++)
	{
		if(mHandlers[i] && &mHandlers[i]->mTimerT35 ==  tim ){
			if(mHandlers[i]->mType == ModBusType::Master)
			{
				osTimerStop(mHandlers[i]->mTimerTimeout);
			}
			xTaskNotify((TaskHandle_t)mHandlers[i]->mTaskHandle, 0, eSetValueWithOverwrite);
		}
	}
}

void Modbus::vTimerCallbackTimeout(void * arg)
{
	osTimerId *tim = (osTimerId*)arg;
	//Notify that a stream has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{
		if( (osTimerId *)&mHandlers[i]->mTimerTimeout ==  tim ){
			xTaskNotify((TaskHandle_t)mHandlers[i]->mTaskHandle, to_underlying(Error::TIME_OUT), eSetValueWithOverwrite);
		}

	}

}
void SlaveTask(void *argument) {
	  Modbus *mH =  (Modbus *)argument;
	  mH->DoSlaveTask();
}

void Modbus::DoSlaveTask()
{
  //uint32_t notification;

  for(;;)
  {

   mLastError = Error::NONE;


	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* Block until a Modbus Frame arrives */

	  if (getRxBuffer() == to_underlying(Error::BUFF_OVERFLOW))
	  {
	      mLastError = Error::BUFF_OVERFLOW;
	   	  mErrCnt++;
		  continue;
	  }


   if (mBufferSize < 7)
   {
      //The size of the frame is invalid
	   mLastError = Error::BAD_SIZE;
	   mErrCnt++;
	  continue;
    }

    //check broadcast mode
   mAddressMode = Address::NORMAL;
    if (mBuffer[to_underlying(Message::ID)] == to_underlying(Address::BROADCAST))
    {
    	mAddressMode = Address::BROADCAST;
    }

   // check slave id
    if ( mBuffer[to_underlying(Message::ID)] !=  mId && mAddressMode != Address::BROADCAST)
	{


    continue; // continue this is not for us

	}

	  // validate message: CRC, FCT, address and size
    uint8_t exception = validateRequest();
	if (exception > 0)
	{
	    if (exception !=  to_underlying(Error::TIME_OUT))
		{
		    buildException( exception);
			sendTxBuffer();
		}
	    mLastError = static_cast<Error>(exception);
		//return u8exception

		continue;
	 }

	mLastError = Error::NONE;
	osSemaphoreAcquire(mSpHandle , osWaitForever); //before processing the message get the semaphore

	 // process message
	 switch(static_cast<FunctionCode>(mBuffer[to_underlying( Message::FUNC) ] ))
	 {
		case FunctionCode::READ_COILS:
		case FunctionCode::READ_DISCRETE_INPUT:
			if (mAddressMode == Address::BROADCAST)
			{
				/* broadcast mode should ignore read function */
				break;
			}
			mState = process_FC1();
			break;
		case FunctionCode::READ_INPUT_REGISTER:
		case FunctionCode::READ_REGISTERS :
			if (mAddressMode == Address::BROADCAST)
			{
				/* broadcast mode should ignore read function */
				break;
			}
			mState = process_FC3();
			break;
		case FunctionCode::WRITE_COIL:
			mState = process_FC5();
			break;
		case FunctionCode::WRITE_REGISTER :
			mState = process_FC6();
			break;
		case FunctionCode::WRITE_MULTIPLE_COILS:
			mState = process_FC15();
			break;
		case FunctionCode::WRITE_MULTIPLE_REGISTERS :
			mState = process_FC16();
			break;
		default:
			break;
	 }


	 osSemaphoreRelease(mSpHandle); //Release the semaphore
	 continue;

   }
}

void Modbus::Query(Query_t q)
{
	//Add the telegram to the TX tail Queue of Modbus
	if (mType == ModBusType::Master)
	{
		q.currentTask = (uint32_t *) osThreadGetId();
		xQueueSendToBack((QueueHandle_t)mQueueMessageHandle, &q, 0);
	}
	else{
		assert(0);
	}
}



void Modbus::QueryInject(Query_t q )
{
	//Add the telegram to the TX head Queue of Modbus
	xQueueReset((QueueHandle_t)mQueueMessageHandle);
	q.currentTask = (uint32_t *) osThreadGetId();
	xQueueSendToFront((QueueHandle_t)mQueueMessageHandle, &q, 0);
}


Error Modbus::SendQuery(Query_t q)
{
	uint8_t regsno, bytesno;
	Error  error = Error::NONE;
	osSemaphoreAcquire(mSpHandle , osWaitForever); //before processing the message get the semaphore

	if (mId!=0) error = Error::NOT_MASTER;
	if (mState != to_underlying(ComState::IDLE)) error = Error::POLLING ;
	if ((q.id==0) || (q.id>247)) error = Error::BAD_SLAVE_ID;


	if(error != Error::NONE)
	{
		mLastError = error;
	    osSemaphoreRelease(mSpHandle);
	    return error;
	}


	mRegs = q.reg;

	// telegram header
	mBuffer[to_underlying(Message::ID) ]         = q.id;
	mBuffer[to_underlying(Message::FUNC) ]       = to_underlying(q.fct);
	mBuffer[to_underlying(Message::ADD_HI) ]     = highByte(q.regAdd );
	mBuffer[to_underlying(Message::ADD_LO) ]     = lowByte( q.regAdd );

	switch( q.fct )
	{
	case FunctionCode::READ_COILS:
	case FunctionCode::READ_DISCRETE_INPUT:
	case FunctionCode::READ_REGISTERS:
	case FunctionCode::READ_INPUT_REGISTER:
	    mBuffer[to_underlying( Message::NB_HI) ]      = highByte(q.coilsNo );
	    mBuffer[to_underlying( Message::NB_LO) ]      = lowByte( q.coilsNo );
	    mBufferSize = 6;
	    break;
	case FunctionCode::WRITE_COIL:
	    mBuffer[to_underlying( Message::NB_HI) ]      = (( q.reg[(Register::Index)0]> 0) ? 0xff : 0);
	    mBuffer[to_underlying( Message::NB_LO) ]      = 0;
	    mBufferSize = 6;
	    break;
	case FunctionCode::WRITE_REGISTER:
	    mBuffer[ to_underlying(Message::NB_HI) ]      = highByte( q.reg[(Register::Index)0]);
	    mBuffer[to_underlying( Message::NB_LO) ]      = lowByte( q.reg[(Register::Index)0]);
	    mBufferSize = 6;
	    break;
	case FunctionCode::WRITE_MULTIPLE_COILS: // TODO: implement "sending coils"
	    regsno = q.coilsNo / 16;
	    bytesno = regsno * 2;
	    if ((q.coilsNo % 16) != 0)
	    {
	        bytesno++;
	        regsno++;
	    }

	    mBuffer[ to_underlying( Message::NB_HI ) ]      = highByte(q.coilsNo );
	    mBuffer[ to_underlying( Message::NB_LO ) ]      = lowByte( q.coilsNo );
	    mBuffer[ to_underlying( Message::BYTE_CNT ) ]    = bytesno;
	    mBufferSize = 7;

	    for (uint16_t i = 0; i < bytesno; i++)
	    {
	        if(i%2)
	        {
	        	mBuffer[ mBufferSize ] = lowByte( q.reg[ (Register::Index)(i/2 )] );
	        }
	        else
	        {
	        	mBuffer[  mBufferSize ] = highByte( q.reg[(Register::Index)( i/2) ] );

	        }
	        mBufferSize++;
	    }
	    break;

	case FunctionCode::WRITE_MULTIPLE_REGISTERS:
	    mBuffer[ to_underlying(Message::NB_HI) ]      = highByte(q.coilsNo );
	    mBuffer[ to_underlying(Message::NB_LO) ]      = lowByte( q.coilsNo );
	    mBuffer[ to_underlying(Message::BYTE_CNT) ]    = (uint8_t) ( q.coilsNo * 2 );
	    mBufferSize = 7;

	    for (uint16_t i=0; i< q.coilsNo; i++)
	    {

	        mBuffer[  mBufferSize ] = highByte(  q.reg[(Register::Index) i ] );
	        mBufferSize++;
	        mBuffer[  mBufferSize ] = lowByte( q.reg[(Register::Index) i ] );
	        mBufferSize++;
	    }
	    break;
	}


	sendTxBuffer();

	xSemaphoreGive(mSpHandle);

	mState = to_underlying(ComState::WAITING);
	mLastError = Error::NONE;
	return  Error::NONE;


}

void MasterTask(void *argument)
{
	Modbus *modH =  (Modbus *)argument;
	modH->DoMasterTask();
}

void Modbus::DoMasterTask()
{

  uint32_t ulNotificationValue;
  Query_t telegram;

  for(;;)
  {
	  if(osMessageQueueGet(mQueueMessageHandle, &telegram, 0, osWaitForever) == osOK) {

     /*Wait period of silence between modbus frame */
	  uint32_t br = UartGetBaudrate(mUart);
	 if(br <= 19200)
	 	osDelay((int)(35000/br) + 2);
	 else
	 	osDelay(3);

     // This is the case for implementations with only USART support
	 SendQuery(telegram);
     /* Block indefinitely until a Modbus Frame arrives or query timeouts*/
     ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	  // notify the task the request timeout
      mLastError = Error::NONE;
      if(ulNotificationValue)
      {
    	  mState = to_underlying(ComState::IDLE);
    	  mLastError = Error::TIME_OUT;
    	  mErrCnt++;
    	  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(mLastError), eSetValueWithOverwrite);
    	  continue;
      }

      getRxBuffer();

	  if ( mBufferSize < 6){

		  mState = to_underlying(ComState::IDLE);
		  mLastError = Error::BAD_SIZE;
		  mErrCnt++;
		  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(mLastError), eSetValueWithOverwrite);
		  continue;
	  }

	  osTimerStop(mTimerTimeout);

	  // validate message: id, CRC, FCT, exception
	  int8_t exception = validateAnswer();
	  if (exception != 0)
	  {
		 mState = to_underlying(ComState::IDLE);
         mLastError = static_cast<Error>(exception);
		 xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(mLastError), eSetValueWithOverwrite);
	     continue;
	  }

	  mLastError = static_cast<Error>(exception);

	  osSemaphoreAcquire(mSpHandle , osWaitForever); //before processing the message get the semaphore
	  // process answer
	  switch(static_cast<FunctionCode>(mBuffer[ to_underlying(Message::FUNC) ] ))
	  {
	  case FunctionCode::READ_COILS:
	  case FunctionCode::READ_DISCRETE_INPUT:
	      //call get_FC1 to transfer the incoming message to u16regs buffer
	      get_FC1();
	      break;
	  case FunctionCode::READ_INPUT_REGISTER:
	  case FunctionCode::READ_REGISTERS :
	      // call get_FC3 to transfer the incoming message to u16regs buffer
	      get_FC3();
	      break;
	  case FunctionCode::WRITE_COIL:
	  case FunctionCode::WRITE_REGISTER :
	  case FunctionCode::WRITE_MULTIPLE_COILS:
	  case FunctionCode::WRITE_MULTIPLE_REGISTERS :
	      // nothing to do
	      break;
	  default:
	      break;
	  }
	  mState = to_underlying(ComState::IDLE);

	  if (mLastError == Error::NONE) // no error the error_OK, we need to use a different value than 0 to detect the timeout
	  {
		  osSemaphoreRelease(mSpHandle); //Release the semaphore
		  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(Error::OK_QUERY), eSetValueWithOverwrite);
	  }


	  continue;
	 }
  }
}

/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void Modbus::get_FC1()
{
    uint8_t byte, i;
    byte = 3;
     for (i=0; i< mBuffer[2]; i++) {

        if(i%2)
        {
        	mRegs[(Register::Index)(i/2)]= word(mBuffer[i+byte], lowByte(mRegs[(Register::Index)(i/2)]));
        }
        else
        {

        	mRegs[(Register::Index)(i/2)]= word(highByte(mRegs[(Register::Index)(i/2)]), mBuffer[i+byte]);
        }

     }
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void Modbus::get_FC3()
{
    uint8_t byte, i;
    byte = 3;

    for (i=0; i< mBuffer[ 2 ] /2; i++)
    {
    	mRegs[  (Register::Index)i ] = word(mBuffer[ byte ], mBuffer[ byte +1 ]);
        byte += 2;
    }
}



/**
 * @brief
 * This method validates master incoming messages
 *
 * @return 0 if OK, EXCEPTION if anything fails
 * @ingroup buffer
 */
uint8_t Modbus::validateAnswer()
{
    // check message crc vs calculated crc


	uint16_t msgCRC =
        ((mBuffer[mBufferSize - 2] << 8)
         | mBuffer[mBufferSize - 1]); // combine the crc Low & High bytes
    if ( calcCRC(mBuffer,  mBufferSize-2) != msgCRC )
    {
    	mErrCnt ++;
        return to_underlying(Error::BAD_CRC);
    }

    // check exception
    if ((mBuffer[ to_underlying(Message::FUNC )] & 0x80) != 0)
    {
    	mErrCnt++;
        return to_underlying(Error::EXCEPTION);
    }

    // check fct code
    bool isSupported = false;
    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == mBuffer[to_underlying(Message::FUNC)])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
    	mErrCnt++;
        return EXC_FUNC_CODE;
    }

    return to_underlying(Error::NONE); // OK, no exception code thrown
}

int16_t Modbus::getRxBuffer()
{

    int16_t result;


    // disable interrupts to avoid race conditions on serial port
    UartAbortReceive(mUart);
	if (mBufferRX.overflow())
    {
       	mBufferRX.Clear(); // clean up the overflowed buffer
       	result =  to_underlying(Error::BUFF_OVERFLOW);
    }
	else
	{
		mBufferSize = mBufferRX.GetAllBytes(mBuffer);
		mInCnt++;
		result = mBufferSize;
	}
	UartGetChar(mUart, &mDataRX, 0);

    return result;
}

uint8_t Modbus::validateRequest()
{
	// check message crc vs calculated crc
	    uint16_t msgCRC;
	    msgCRC= ((mBuffer[mBufferSize - 2] << 8)
	    		   	         | mBuffer[mBufferSize - 1]); // combine the crc Low & High bytes


	    if ( calcCRC( mBuffer,  mBufferSize-2 ) != msgCRC )
	    {
	       		mErrCnt ++;
	       		return to_underlying(Error::BAD_CRC);
	    }




	    // check fct code
	    bool isSupported = false;
	    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
	    {
	        if (fctsupported[i] == mBuffer[to_underlying(Message::FUNC)])
	        {
	            isSupported = 1;
	            break;
	        }
	    }
	    if (!isSupported)
	    {
	    	mErrCnt++;
	        return EXC_FUNC_CODE;
	    }

	    // check start address & nb range
	    uint16_t u16AdRegs = 0;
	    uint16_t u16NRegs = 0;

	    //uint8_t u8regs;
	    switch ( static_cast<FunctionCode>(mBuffer[ to_underlying(Message::FUNC) ]) )
	    {
	    case FunctionCode::READ_COILS:
	    case FunctionCode::READ_DISCRETE_INPUT:
	    case FunctionCode::WRITE_MULTIPLE_COILS:
	    	u16AdRegs = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ]) / 16;
	    	u16NRegs = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ]) /16;
	    	if(word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ]) % 16) u16NRegs++; // check for incomplete words
	    	// verify address range
	    	if((u16AdRegs + u16NRegs) > mRegs.size()) return EXC_ADDR_RANGE;

	    	//verify answer frame size in bytes

	    	u16NRegs = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ]) / 8;
	    	if(word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ]) % 8) u16NRegs++;
	    	u16NRegs = u16NRegs + 5; // adding the header  and CRC ( Slave address + Function code  + number of data bytes to follow + 2-byte CRC )
	        if(u16NRegs > 256) return EXC_REGS_QUANT;

	        break;
	    case FunctionCode::WRITE_COIL:
	    	u16AdRegs = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ]) / 16;
	    	if(word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[to_underlying( Message::ADD_LO) ]) % 16) u16AdRegs++;	// check for incomplete words
	        if (u16AdRegs > mRegs.size()) return EXC_ADDR_RANGE;
	        break;
	    case FunctionCode::WRITE_REGISTER :
	    	u16AdRegs = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ]);
	        if (u16AdRegs >  mRegs.size()) return EXC_ADDR_RANGE;
	        break;
	    case FunctionCode::READ_REGISTERS :
	    case FunctionCode::READ_INPUT_REGISTER :
	    case FunctionCode::WRITE_MULTIPLE_REGISTERS :
	    	u16AdRegs = word( mBuffer[to_underlying( Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ]);
	        u16NRegs = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ]);
	        if (( u16AdRegs + u16NRegs ) > mRegs.size()) return EXC_ADDR_RANGE;

	        //verify answer frame size in bytes
	        u16NRegs = u16NRegs*2 + 5; // adding the header  and CRC
	        if ( u16NRegs > 256 ) return EXC_REGS_QUANT;
	        break;
	    }
	    return 0; // OK, no exception code thrown

}

/**
 * @brief
 * This method creates a word from 2 bytes
 *
 * @return uint16_t (word)
 * @ingroup H  Most significant byte
 * @ingroup L  Less significant byte
 */
uint16_t Modbus::word(uint8_t H, uint8_t L)
{
	byteFields W;
	W.u8[0] = L;
	W.u8[1] = H;

	return W.u16[0];
}


/**
 * @brief
 * This method calculates CRC
 *
 * @return uint16_t calculated CRC value for the message
 * @ingroup Buffer
 * @ingroup u8length
 */
uint16_t Modbus::calcCRC(uint8_t *Buffer, uint8_t u8length)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < u8length; i++)
    {
        temp = temp ^ Buffer[i];
        for (unsigned char j = 1; j <= 8; j++)
        {
            flag = temp & 0x0001;
            temp >>=1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    // Reverse byte order.
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    // the returned value is already swapped
    // crcLo byte is first & crcHi byte is last
    return temp;

}


/**
 * @brief
 * This method builds an exception message
 *
 * @ingroup u8exception exception number
 * @ingroup modH modbus handler
 */
void Modbus::buildException( uint8_t exception)
{
    uint8_t func = mBuffer[ to_underlying(Message::FUNC) ];  // get the original FUNC code

    mBuffer[ to_underlying(Message::ID) ]      = mId;
    mBuffer[ to_underlying(Message::FUNC) ]    = func + 0x80;
    mBuffer[ 2 ]       = exception;
    mBufferSize         = to_underlying(PACKET_SIZE::EXCEPTION);
}

/**
 * @brief
 * This method transmits u8Buffer to Serial line.
 * Only if u8txenpin != 0, there is a flow handling in order to keep
 * the RS485 transceiver in output state as long as the message is being sent.
 * This is done with TC bit.
 * The CRC is appended to the buffer before starting to send it.
 *
 * @return nothing
 * @ingroup modH Modbus handler
 */
void Modbus::sendTxBuffer()
{
    // when in slaveType and u8AddressMode == ADDRESS_BROADCAST, do not send anything
    if (mType == ModBusType::Slave && mAddressMode == Address::BROADCAST)
    {
        mBufferSize = 0;
        // increase message counter
        mOutCnt++;
        return;
    }

    // append CRC to message

	uint16_t crc = calcCRC(mBuffer, mBufferSize);
    mBuffer[ mBufferSize ] = crc >> 8;
    mBufferSize++;
    mBuffer[ mBufferSize ] = crc & 0x00ff;
    mBufferSize++;


    	if (mDePin != nullptr)
        {

    		//enable transmitter, disable receiver to avoid echo on RS485 transceivers
    		UartEnableTransmitter(mUart);
    		GpioWrite( mDePin, 1);
        }


        	//transfer buffer to serial line
        UartPutBuffer(mUart, mBuffer, mBufferSize, 0);

        ulTaskNotifyTake(pdTRUE, 250); //wait notification from TXE interrupt
/*
* If you are porting the library to a different MCU check the
* USART datasheet and add the corresponding family in the following
* preprocessor conditions
*/
        UartLastByteSendOut(mUart, TIMEOUT_MODBUS);


         if (mDePin != nullptr)
         {

             //return RS485 transceiver to receive mode
        	 GpioWrite(  mDePin, 0);
        	 //enable receiver, disable transmitter
        	 UartEnableReciever(mUart);

         }

         // set timeout for master query
         if(mType == ModBusType::Master )
         {
        	 osTimerStart(mTimerTimeout, mTimeOut);
         }
     mBufferSize = 0;
     // increase message counter
     mOutCnt++;


}

int8_t Modbus::process_FC1()
{
    uint16_t currentRegister;
    uint8_t currentBit, bytesno, bitsno;
    uint8_t copyBufferSize;
    uint16_t currentCoil, coil;

    // get the first and last coil from the message
    uint16_t startCoil = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint16_t coilno = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ] );

    // put the number of bytes in the outcoming message
    bytesno = (uint8_t) (coilno / 8);
    if (coilno % 8 != 0) bytesno ++;
    mBuffer[ to_underlying(Message::ADD_HI) ]  = bytesno;
    mBufferSize         = to_underlying(Message::ADD_LO);
    mBuffer[mBufferSize + bytesno - 1 ] = 0;

    // read each coil from the register map and put its value inside the outcoming message
    bitsno = 0;

    for (currentCoil = 0; currentCoil < coilno; currentCoil++)
    {
        coil = startCoil + currentCoil;
        currentRegister =  (coil / 16);
        currentBit = (uint8_t) (coil % 16);

        bitWrite(
        	mBuffer[ mBufferSize ],
            bitsno,
		    bitRead( mRegs[ (Register::Index)currentRegister ], currentBit ) );
        bitsno ++;

        if (bitsno > 7)
        {
            bitsno = 0;
            mBufferSize++;
        }
    }

    // send outcoming message
    if (coilno % 8 != 0) mBufferSize ++;
    copyBufferSize = mBufferSize +2;
    sendTxBuffer();
    return copyBufferSize;
}


/**
 * @brief
 * This method processes functions 3 & 4
 * This method reads a word array and transfers it to the master
 *
 * @return u8BufferSize Response to master length
 * @ingroup register
 */
int8_t Modbus::process_FC3()
{

    uint16_t startAdd = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint8_t regsno = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ] );
    uint8_t copyBufferSize;
    uint16_t i;

    mBuffer[ 2 ]       = regsno * 2;
    mBufferSize         = 3;

    for (i = startAdd; i < startAdd + regsno; i++)
    {
    	mBuffer[ mBufferSize ] = highByte(mRegs[ (Register::Index)i]);
    	mBufferSize++;
    	mBuffer[ mBufferSize ] = lowByte(mRegs[ (Register::Index)i]);
    	mBufferSize++;
    }
    copyBufferSize = mBufferSize +2;
    sendTxBuffer();

    return copyBufferSize;
}

int8_t Modbus::process_FC5()
{
    uint8_t currentBit;
    uint16_t currentRegister;
    uint8_t copyBufferSize;
    uint16_t coil = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ] );

    // point to the register and its bit
    currentRegister = (coil / 16);
    currentBit = (uint8_t) (coil % 16);

    // write to coil
    bitWrite(
    	mRegs[  (Register::Index)currentRegister ],
        currentBit,
		mBuffer[ to_underlying(Message::NB_HI) ] == 0xff );


    // send answer to master
    mBufferSize = 6;
    copyBufferSize =  mBufferSize +2;
    sendTxBuffer();

    return copyBufferSize;
}

int8_t Modbus::process_FC6()
{

    uint16_t add = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint8_t copyBufferSize;
    uint16_t val = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ] );

    mRegs[ (Register::Index) add ] = val;

    // keep the same header
    mBufferSize = to_underlying(PACKET_SIZE::RESPONSE);

    copyBufferSize = mBufferSize + 2;
    sendTxBuffer();

    return copyBufferSize;
}

int8_t Modbus::process_FC15()
{
    uint8_t currentBit, frameByte, bitsno;
    uint16_t currentRegister;
    uint8_t copyBufferSize;
    uint16_t currentCoil, coil;
    bool bTemp;

    // get the first and last coil from the message
    uint16_t startCoil = word( mBuffer[ to_underlying(Message::ADD_HI) ], mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint16_t coilno = word( mBuffer[ to_underlying(Message::NB_HI) ], mBuffer[ to_underlying(Message::NB_LO) ] );


    // read each coil from the register map and put its value inside the outcoming message
    bitsno = 0;
    frameByte = 7;
    for (currentCoil = 0; currentCoil < coilno; currentCoil++)
    {

        coil = startCoil + currentCoil;
        currentRegister = (coil / 16);
        currentBit = (uint8_t) (coil % 16);

        bTemp = bitRead(
        			mBuffer[ frameByte ],
                    bitsno );
        auto &reg = mRegs[ (Register::Index) currentRegister ];
        bitWrite(
            reg,
            currentBit,
            bTemp );

        bitsno ++;

        if (bitsno > 7)
        {
            bitsno = 0;
            frameByte++;
        }
    }

    // send outcoming message
    // it's just a copy of the incomping frame until 6th byte
    mBufferSize         = 6;
    copyBufferSize = mBufferSize +2;
    sendTxBuffer();
    return copyBufferSize;
}

int8_t Modbus::process_FC16()
{
    uint16_t startAdd = mBuffer[ to_underlying(Message::ADD_HI) ] << 8 | mBuffer[ to_underlying(Message::ADD_LO) ];
    uint16_t regsno = mBuffer[ to_underlying(Message::NB_HI) ] << 8 | mBuffer[ to_underlying(Message::NB_LO) ];
    uint8_t copyBufferSize;
    uint16_t i;
    uint16_t temp;

    // build header
    mBuffer[ to_underlying(Message::NB_HI) ]   = 0;
    mBuffer[ to_underlying(Message::NB_LO) ]   = (uint8_t) regsno; // answer is always 256 or less bytes
    mBufferSize         = to_underlying(PACKET_SIZE::RESPONSE);

    // write registers
    for (i = 0; i < regsno; i++)
    {
        temp = word(
        		mBuffer[ (to_underlying(Message::BYTE_CNT) + 1) + i * 2 ],
				mBuffer[ (to_underlying(Message::BYTE_CNT) + 2) + i * 2 ]);
        if(mRegs.find((Register::Index)(startAdd + i)) != mRegs.end()) {
        	mRegs[(Register::Index) (startAdd + i) ] = temp;
        }
    }
    copyBufferSize = mBufferSize +2;
    sendTxBuffer();

    return copyBufferSize;
}
}
