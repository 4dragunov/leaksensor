/*
 * modbus.cpp
 *
 *  Created on: Nov 27, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */
#include <type_traits>
#include <cassert>

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

ModbusHandler *mHandlers[MAX_M_HANDLERS];


///Queue Modbus telegrams for master
osMessageQDef(ModbusMessages, MAX_M_HANDLERS, 0);


osThreadDef(ModbusMasterTask, ModbusHandler::MasterTask, osPriorityNormal, MAX_M_HANDLERS, 128 * 4);

osThreadDef(ModbusSlaveTask, ModbusHandler::SlaveTask, osPriorityNormal, MAX_M_HANDLERS, 128 * 4);


//Semaphore to access the Modbus Data
osSemaphoreDef(ModBusSp);

uint8_t numberHandlers = 0;

extern "C" void vTimerCallbackT35(TimerHandle_t *pxTimer);
extern "C" void vTimerCallbackTimeout(TimerHandle_t *pxTimer);

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



ModbusHandler::ModbusHandler(Uart_t *uart, Gpio_t *dePin, ModBusType type, const uint8_t id, Registers &regs):
		mType(type),
		mUart(uart),
		mId(1, 254, id, NvVar::MODBUS_SLAVE_ID), //0 - only for master
		mBaudRate(2400, 115200, 19200, NvVar::MODBUS_SPEED),
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

     assert(numberHandlers < MAX_M_HANDLERS);
  	  //Initialize the ring buffer

	 mBufferRX.Clear();

	 if(mType == ModBusType::Slave)
	 {
		  //Create Modbus task slave

		  mTaskHandle =  osThreadCreate(osThread(ModbusSlaveTask), this);

	 }
	 else if (mType == ModBusType::Master)
	 {
		 mTaskHandle =  osThreadCreate(osThread(ModbusMasterTask), this);

		 mTimerTimeout = xTimerCreate("xTimerTimeout",  // Just a text name, not used by the kernel.
				  	  	mTimeOut ,     		// The timer period in ticks.
						pdFALSE,         // The timers will auto-reload themselves when they expire.
						( void * )mTimerTimeout,     // Assign each timer a unique id equal to its array index.
						(TimerCallbackFunction_t) vTimerCallbackTimeout  // Each timer calls the same callback when it expires.
                  	  	);

		 if(mTimerTimeout == NULL)
		 {
			  while(1); //error creating timer, check heap and stack size
		 }

		 mQueueMessageHandle = osMessageCreate(osMessageQ(ModbusMessages), NULL);

		 if(mQueueMessageHandle == NULL)
		 {
			  while(1); //error creating queue for telegrams, check heap and stack size
		 }

	  }
	  else
	  {
		  while(1); //Error Modbus type not supported choose a valid Type
	  }

	  if(mTaskHandle == NULL)
	  {
		  while(1); //Error creating Modbus task, check heap and stack size
	  }


	  assert(mTimerT35 = xTimerCreate("TimerT35",         // Just a text name, not used by the kernel.
		  	  	  	  	  	  	  	T35 ,     // The timer period in ticks.
                                    pdFALSE,         // The timers will auto-reload themselves when they expire.
									( void * )mTimerT35,     // Assign each timer a unique id equal to its array index.
                                    (TimerCallbackFunction_t) vTimerCallbackT35     // Each timer calls the same callback when it expires.
                                    ));


	  assert(mSpHandle = osSemaphoreCreate(osSemaphore(ModBusSp), 1));


	  mHandlers[numberHandlers++] = this;
}

void ModbusHandler::SetLine()
{
	UartConfig( mUart, RX_TX, SYNC, mBaudRate, static_cast<WordLength_t>((uint8_t)mWordLen),static_cast<StopBits_t>((uint8_t)mStopBits), static_cast<Parity_t>((uint8_t)mParity), NO_FLOW_CTRL );
}

void ModbusHandler::Start()
{
	SetLine();
	if (mDePin != NULL )
    {
       // return RS485 transceiver to receive mode
		GpioWrite(mDePin, 0);
    }

    if (mType == ModBusType::Slave  &&  mRegs.empty())
    {
      while(1); //ERROR define the DATA pointer shared through Modbus
    }

    //check that port is initialized

    assert(UartWaitReady(mUart, TIMEOUT_MODBUS));

    UartGetBufferToIdle(mUart, (uint8_t*)mBufferRX.buffer(), MAX_BUFFER, 0, TIMEOUT_MODBUS);

    assert((mId ==0 && mType == ModBusType::Master ) || (mId !=0 && mType == ModBusType::Slave ));

    mLastRec = mBufferSize = 0;
    mInCnt = mOutCnt = mErrCnt = 0;
}


void ModbusHandler::vTimerCallbackT35(TimerHandle_t *pxTimer)
{
	//Notify that a stream has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{
		if(mHandlers[i] && (TimerHandle_t *)mHandlers[i]->mTimerT35 ==  pxTimer ){
			if(mHandlers[i]->mType == ModBusType::Master)
			{
				xTimerStop(mHandlers[i]->mTimerTimeout, 0);
			}
			xTaskNotify(mHandlers[i]->mTaskHandle, 0, eSetValueWithOverwrite);
		}
	}
}

void ModbusHandler::vTimerCallbackTimeout(TimerHandle_t *pxTimer)
{
	//Notify that a stream has just arrived
	int i;
	//TimerHandle_t aux;
	for(i = 0; i < numberHandlers; i++)
	{
		if( (TimerHandle_t *)mHandlers[i]->mTimerTimeout ==  pxTimer ){
			xTaskNotify(mHandlers[i]->mTaskHandle, to_underlying(Error::TIME_OUT), eSetValueWithOverwrite);
		}

	}

}


void ModbusHandler::SlaveTask(const void *argument)
{

  ModbusHandler *mH =  (ModbusHandler *)argument;
  //uint32_t notification;

  for(;;)
  {

   mH->mLastError = Error::NONE;


	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* Block until a Modbus Frame arrives */

	  if (mH->getRxBuffer() == to_underlying(Error::BUFF_OVERFLOW))
	  {
	      mH->mLastError = Error::BUFF_OVERFLOW;
	   	  mH->mErrCnt++;
		  continue;
	  }


   if (mH->mBufferSize < 7)
   {
      //The size of the frame is invalid
	   mH->mLastError = Error::BAD_SIZE;
	   mH->mErrCnt++;
	  continue;
    }

    //check broadcast mode
   mH->mAddressMode = Address::NORMAL;
    if (mH->mBuffer[to_underlying(Message::ID)] == to_underlying(Address::BROADCAST))
    {
    	mH->mAddressMode = Address::BROADCAST;
    }

   // check slave id
    if ( mH->mBuffer[to_underlying(Message::ID)] !=  mH->mId && mH->mAddressMode != Address::BROADCAST)
	{


    continue; // continue this is not for us

	}

	  // validate message: CRC, FCT, address and size
    uint8_t exception = validateRequest(mH);
	if (exception > 0)
	{
	    if (exception !=  to_underlying(Error::TIME_OUT))
		{
		    buildException( exception, mH);
			sendTxBuffer(mH);
		}
	    mH->mLastError = static_cast<Error>(exception);
		//return u8exception

		continue;
	 }

	mH->mLastError = Error::NONE;
	xSemaphoreTake(mH->mSpHandle , portMAX_DELAY); //before processing the message get the semaphore

	 // process message
	 switch(static_cast<FunctionCode>(mH->mBuffer[to_underlying( Message::FUNC) ] ))
	 {
		case FunctionCode::READ_COILS:
		case FunctionCode::READ_DISCRETE_INPUT:
			if (mH->mAddressMode == Address::BROADCAST)
			{
				/* broadcast mode should ignore read function */
				break;
			}
			mH->mState = process_FC1(mH);
			break;
		case FunctionCode::READ_INPUT_REGISTER:
		case FunctionCode::READ_REGISTERS :
			if (mH->mAddressMode == Address::BROADCAST)
			{
				/* broadcast mode should ignore read function */
				break;
			}
			mH->mState = process_FC3(mH);
			break;
		case FunctionCode::WRITE_COIL:
			mH->mState = process_FC5(mH);
			break;
		case FunctionCode::WRITE_REGISTER :
			mH->mState = process_FC6(mH);
			break;
		case FunctionCode::WRITE_MULTIPLE_COILS:
			mH->mState = process_FC15(mH);
			break;
		case FunctionCode::WRITE_MULTIPLE_REGISTERS :
			mH->mState = process_FC16(mH);
			break;
		default:
			break;
	 }


	 xSemaphoreGive(mH->mSpHandle); //Release the semaphore
	 continue;

   }
}

void ModbusHandler::Query(Query_t q)
{
	//Add the telegram to the TX tail Queue of Modbus
	if (mType == ModBusType::Master)
	{
		q.currentTask = (uint32_t *) osThreadGetId();
		xQueueSendToBack(mQueueMessageHandle, &q, 0);
	}
	else{
		assert(0);
	}
}



void ModbusHandler::QueryInject(Query_t q )
{
	//Add the telegram to the TX head Queue of Modbus
	xQueueReset(mQueueMessageHandle);
	q.currentTask = (uint32_t *) osThreadGetId();
	xQueueSendToFront(mQueueMessageHandle, &q, 0);
}


Error ModbusHandler::SendQuery(Query_t q)
{


	uint8_t regsno, bytesno;
	Error  error = Error::NONE;
	xSemaphoreTake(mSpHandle , portMAX_DELAY); //before processing the message get the semaphore

	if (mId!=0) error = Error::NOT_MASTER;
	if (mState != to_underlying(ComState::IDLE)) error = Error::POLLING ;
	if ((q.id==0) || (q.id>247)) error = Error::BAD_SLAVE_ID;


	if(error != Error::NONE)
	{
		mLastError = error;
	    xSemaphoreGive(mSpHandle);
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
	    mBuffer[to_underlying( Message::NB_HI) ]      = (( q.reg[(ModbusRegisterIndex)0]> 0) ? 0xff : 0);
	    mBuffer[to_underlying( Message::NB_LO) ]      = 0;
	    mBufferSize = 6;
	    break;
	case FunctionCode::WRITE_REGISTER:
	    mBuffer[ to_underlying(Message::NB_HI) ]      = highByte( q.reg[(ModbusRegisterIndex)0]);
	    mBuffer[to_underlying( Message::NB_LO) ]      = lowByte( q.reg[(ModbusRegisterIndex)0]);
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
	        	mBuffer[ mBufferSize ] = lowByte( q.reg[ (ModbusRegisterIndex)(i/2 )] );
	        }
	        else
	        {
	        	mBuffer[  mBufferSize ] = highByte( q.reg[(ModbusRegisterIndex)( i/2) ] );

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

	        mBuffer[  mBufferSize ] = highByte(  q.reg[(ModbusRegisterIndex) i ] );
	        mBufferSize++;
	        mBuffer[  mBufferSize ] = lowByte( q.reg[(ModbusRegisterIndex) i ] );
	        mBufferSize++;
	    }
	    break;
	}


	sendTxBuffer(this);

	xSemaphoreGive(mSpHandle);

	mState = to_underlying(ComState::WAITING);
	mLastError = Error::NONE;
	return  Error::NONE;


}

void ModbusHandler::MasterTask(const void *argument)
{

  ModbusHandler *modH =  (ModbusHandler *)argument;
  uint32_t ulNotificationValue;
  Query_t telegram;



  for(;;)
  {
	  /*Wait indefinitely for a telegram to send */
	  xQueueReceive(modH->mQueueMessageHandle, &telegram, portMAX_DELAY);



     /*Wait period of silence between modbus frame */
	  uint32_t br = UartGetBaudrate(modH->mUart);
	 if(br <= 19200)
	 	osDelay((int)(35000/br) + 2);
	 else
	 	osDelay(3);

     // This is the case for implementations with only USART support
	 modH->SendQuery(telegram);
     /* Block indefinitely until a Modbus Frame arrives or query timeouts*/
     ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	  // notify the task the request timeout
      modH->mLastError = Error::NONE;
      if(ulNotificationValue)
      {
    	  modH->mState = to_underlying(ComState::IDLE);
    	  modH->mLastError = Error::TIME_OUT;
    	  modH->mErrCnt++;
    	  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(modH->mLastError), eSetValueWithOverwrite);
    	  continue;
      }

      modH->getRxBuffer();

	  if ( modH->mBufferSize < 6){

		  modH->mState = to_underlying(ComState::IDLE);
		  modH->mLastError = Error::BAD_SIZE;
		  modH->mErrCnt++;
		  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(modH->mLastError), eSetValueWithOverwrite);
		  continue;
	  }

	  xTimerStop(modH->mTimerTimeout,0); // cancel timeout timer


	  // validate message: id, CRC, FCT, exception
	  int8_t exception = validateAnswer(modH);
	  if (exception != 0)
	  {
		 modH->mState = to_underlying(ComState::IDLE);
         modH->mLastError = static_cast<Error>(exception);
		 xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(modH->mLastError), eSetValueWithOverwrite);
	     continue;
	  }

	  modH->mLastError = static_cast<Error>(exception);

	  xSemaphoreTake(modH->mSpHandle , portMAX_DELAY); //before processing the message get the semaphore
	  // process answer
	  switch(static_cast<FunctionCode>(modH->mBuffer[ to_underlying(Message::FUNC) ] ))
	  {
	  case FunctionCode::READ_COILS:
	  case FunctionCode::READ_DISCRETE_INPUT:
	      //call get_FC1 to transfer the incoming message to u16regs buffer
	      get_FC1(modH);
	      break;
	  case FunctionCode::READ_INPUT_REGISTER:
	  case FunctionCode::READ_REGISTERS :
	      // call get_FC3 to transfer the incoming message to u16regs buffer
	      get_FC3(modH);
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
	  modH->mState = to_underlying(ComState::IDLE);

	  if (modH->mLastError == Error::NONE) // no error the error_OK, we need to use a different value than 0 to detect the timeout
	  {
		  xSemaphoreGive(modH->mSpHandle); //Release the semaphore
		  xTaskNotify((TaskHandle_t)telegram.currentTask, to_underlying(Error::OK_QUERY), eSetValueWithOverwrite);
	  }


	  continue;
	 }

}

/**
 * This method processes functions 1 & 2 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void ModbusHandler::get_FC1(ModbusHandler *modH)
{
    uint8_t byte, i;
    byte = 3;
     for (i=0; i< modH->mBuffer[2]; i++) {

        if(i%2)
        {
        	modH->mRegs[(ModbusRegisterIndex)(i/2)]= word(modH->mBuffer[i+byte], lowByte(modH->mRegs[(ModbusRegisterIndex)(i/2)]));
        }
        else
        {

        	modH->mRegs[(ModbusRegisterIndex)(i/2)]= word(highByte(modH->mRegs[(ModbusRegisterIndex)(i/2)]), modH->mBuffer[i+byte]);
        }

     }
}

/**
 * This method processes functions 3 & 4 (for master)
 * This method puts the slave answer into master data buffer
 *
 * @ingroup register
 */
void ModbusHandler::get_FC3(ModbusHandler *modH)
{
    uint8_t byte, i;
    byte = 3;

    for (i=0; i< modH->mBuffer[ 2 ] /2; i++)
    {
    	modH->mRegs[  (ModbusRegisterIndex)i ] = word(modH->mBuffer[ byte ], modH->mBuffer[ byte +1 ]);
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
uint8_t ModbusHandler::validateAnswer(ModbusHandler *modH)
{
    // check message crc vs calculated crc


	uint16_t msgCRC =
        ((modH->mBuffer[modH->mBufferSize - 2] << 8)
         | modH->mBuffer[modH->mBufferSize - 1]); // combine the crc Low & High bytes
    if ( calcCRC(modH->mBuffer,  modH->mBufferSize-2) != msgCRC )
    {
    	modH->mErrCnt ++;
        return to_underlying(Error::BAD_CRC);
    }

    // check exception
    if ((modH->mBuffer[ to_underlying(Message::FUNC )] & 0x80) != 0)
    {
    	modH->mErrCnt++;
        return to_underlying(Error::EXCEPTION);
    }

    // check fct code
    bool isSupported = false;
    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
    {
        if (fctsupported[i] == modH->mBuffer[to_underlying(Message::FUNC)])
        {
            isSupported = 1;
            break;
        }
    }
    if (!isSupported)
    {
    	modH->mErrCnt++;
        return EXC_FUNC_CODE;
    }

    return to_underlying(Error::NONE); // OK, no exception code thrown
}

int16_t ModbusHandler::getRxBuffer()
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

uint8_t ModbusHandler::validateRequest(ModbusHandler *modH)
{
	// check message crc vs calculated crc
	    uint16_t msgCRC;
	    msgCRC= ((modH->mBuffer[modH->mBufferSize - 2] << 8)
	    		   	         | modH->mBuffer[modH->mBufferSize - 1]); // combine the crc Low & High bytes


	    if ( calcCRC( modH->mBuffer,  modH->mBufferSize-2 ) != msgCRC )
	    {
	       		modH->mErrCnt ++;
	       		return to_underlying(Error::BAD_CRC);
	    }




	    // check fct code
	    bool isSupported = false;
	    for (uint8_t i = 0; i< sizeof( fctsupported ); i++)
	    {
	        if (fctsupported[i] == modH->mBuffer[to_underlying(Message::FUNC)])
	        {
	            isSupported = 1;
	            break;
	        }
	    }
	    if (!isSupported)
	    {
	    	modH->mErrCnt++;
	        return EXC_FUNC_CODE;
	    }

	    // check start address & nb range
	    uint16_t u16AdRegs = 0;
	    uint16_t u16NRegs = 0;

	    //uint8_t u8regs;
	    switch ( static_cast<FunctionCode>(modH->mBuffer[ to_underlying(Message::FUNC) ]) )
	    {
	    case FunctionCode::READ_COILS:
	    case FunctionCode::READ_DISCRETE_INPUT:
	    case FunctionCode::WRITE_MULTIPLE_COILS:
	    	u16AdRegs = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ]) / 16;
	    	u16NRegs = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ]) /16;
	    	if(word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ]) % 16) u16NRegs++; // check for incomplete words
	    	// verify address range
	    	if((u16AdRegs + u16NRegs) > modH->mRegs.size()) return EXC_ADDR_RANGE;

	    	//verify answer frame size in bytes

	    	u16NRegs = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ]) / 8;
	    	if(word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ]) % 8) u16NRegs++;
	    	u16NRegs = u16NRegs + 5; // adding the header  and CRC ( Slave address + Function code  + number of data bytes to follow + 2-byte CRC )
	        if(u16NRegs > 256) return EXC_REGS_QUANT;

	        break;
	    case FunctionCode::WRITE_COIL:
	    	u16AdRegs = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ]) / 16;
	    	if(word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[to_underlying( Message::ADD_LO) ]) % 16) u16AdRegs++;	// check for incomplete words
	        if (u16AdRegs > modH->mRegs.size()) return EXC_ADDR_RANGE;
	        break;
	    case FunctionCode::WRITE_REGISTER :
	    	u16AdRegs = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ]);
	        if (u16AdRegs > modH-> mRegs.size()) return EXC_ADDR_RANGE;
	        break;
	    case FunctionCode::READ_REGISTERS :
	    case FunctionCode::READ_INPUT_REGISTER :
	    case FunctionCode::WRITE_MULTIPLE_REGISTERS :
	    	u16AdRegs = word( modH->mBuffer[to_underlying( Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ]);
	        u16NRegs = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ]);
	        if (( u16AdRegs + u16NRegs ) > modH->mRegs.size()) return EXC_ADDR_RANGE;

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
uint16_t ModbusHandler::word(uint8_t H, uint8_t L)
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
uint16_t ModbusHandler::calcCRC(uint8_t *Buffer, uint8_t u8length)
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
void ModbusHandler::buildException( uint8_t exception, ModbusHandler *modH )
{
    uint8_t func = modH->mBuffer[ to_underlying(Message::FUNC) ];  // get the original FUNC code

    modH->mBuffer[ to_underlying(Message::ID) ]      = modH->mId;
    modH->mBuffer[ to_underlying(Message::FUNC) ]    = func + 0x80;
    modH->mBuffer[ 2 ]       = exception;
    modH->mBufferSize         = to_underlying(PACKET_SIZE::EXCEPTION);
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
void ModbusHandler::sendTxBuffer(ModbusHandler *modH)
{
    // when in slaveType and u8AddressMode == ADDRESS_BROADCAST, do not send anything
    if (modH->mType == ModBusType::Slave && modH->mAddressMode == Address::BROADCAST)
    {
        modH->mBufferSize = 0;
        // increase message counter
        modH->mOutCnt++;
        return;
    }

    // append CRC to message

	uint16_t crc = calcCRC(modH->mBuffer, modH->mBufferSize);
    modH->mBuffer[ modH->mBufferSize ] = crc >> 8;
    modH->mBufferSize++;
    modH->mBuffer[ modH->mBufferSize ] = crc & 0x00ff;
    modH->mBufferSize++;


    	if (modH->mDePin != NULL)
        {

    		//enable transmitter, disable receiver to avoid echo on RS485 transceivers
    		UartEnableTransmitter(modH->mUart);
    		GpioWrite( modH->mDePin, 1);
        }


        	//transfer buffer to serial line DMA
        UartPutBuffer(modH->mUart, modH->mBuffer, modH->mBufferSize, 0);

        ulTaskNotifyTake(pdTRUE, 250); //wait notification from TXE interrupt
/*
* If you are porting the library to a different MCU check the
* USART datasheet and add the corresponding family in the following
* preprocessor conditions
*/
        UartLastByteSendOut(modH->mUart, TIMEOUT_MODBUS);


         if (modH->mDePin != NULL)
         {

             //return RS485 transceiver to receive mode
        	 GpioWrite(  modH->mDePin, 0);
        	 //enable receiver, disable transmitter
        	 UartEnableReciever(modH->mUart);

         }

         // set timeout for master query
         if(modH->mType == ModBusType::Master )
         {
        	 xTimerReset(modH->mTimerTimeout,0);
         }
     modH->mBufferSize = 0;
     // increase message counter
     modH->mOutCnt++;


}

int8_t ModbusHandler::process_FC1(ModbusHandler *modH )
{
    uint16_t currentRegister;
    uint8_t currentBit, bytesno, bitsno;
    uint8_t copyBufferSize;
    uint16_t currentCoil, coil;

    // get the first and last coil from the message
    uint16_t startCoil = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint16_t coilno = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ] );

    // put the number of bytes in the outcoming message
    bytesno = (uint8_t) (coilno / 8);
    if (coilno % 8 != 0) bytesno ++;
    modH->mBuffer[ to_underlying(Message::ADD_HI) ]  = bytesno;
    modH->mBufferSize         = to_underlying(Message::ADD_LO);
    modH->mBuffer[modH->mBufferSize + bytesno - 1 ] = 0;

    // read each coil from the register map and put its value inside the outcoming message
    bitsno = 0;

    for (currentCoil = 0; currentCoil < coilno; currentCoil++)
    {
        coil = startCoil + currentCoil;
        currentRegister =  (coil / 16);
        currentBit = (uint8_t) (coil % 16);

        bitWrite(
        	modH->mBuffer[ modH->mBufferSize ],
            bitsno,
		    bitRead( modH->mRegs[ (ModbusRegisterIndex)currentRegister ], currentBit ) );
        bitsno ++;

        if (bitsno > 7)
        {
            bitsno = 0;
            modH->mBufferSize++;
        }
    }

    // send outcoming message
    if (coilno % 8 != 0) modH->mBufferSize ++;
    copyBufferSize = modH->mBufferSize +2;
    sendTxBuffer(modH);
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
int8_t ModbusHandler::process_FC3(ModbusHandler *modH)
{

    uint16_t startAdd = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint8_t regsno = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ] );
    uint8_t copyBufferSize;
    uint16_t i;

    modH->mBuffer[ 2 ]       = regsno * 2;
    modH->mBufferSize         = 3;

    for (i = startAdd; i < startAdd + regsno; i++)
    {
    	modH->mBuffer[ modH->mBufferSize ] = highByte(modH->mRegs[ (ModbusRegisterIndex)i]);
    	modH->mBufferSize++;
    	modH->mBuffer[ modH->mBufferSize ] = lowByte(modH->mRegs[ (ModbusRegisterIndex)i]);
    	modH->mBufferSize++;
    }
    copyBufferSize = modH->mBufferSize +2;
    sendTxBuffer(modH);

    return copyBufferSize;
}

int8_t ModbusHandler::process_FC5( ModbusHandler *modH )
{
    uint8_t currentBit;
    uint16_t currentRegister;
    uint8_t copyBufferSize;
    uint16_t coil = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ] );

    // point to the register and its bit
    currentRegister = (coil / 16);
    currentBit = (uint8_t) (coil % 16);

    // write to coil
    bitWrite(
    	modH->mRegs[  (ModbusRegisterIndex)currentRegister ],
        currentBit,
		modH->mBuffer[ to_underlying(Message::NB_HI) ] == 0xff );


    // send answer to master
    modH->mBufferSize = 6;
    copyBufferSize =  modH->mBufferSize +2;
    sendTxBuffer(modH);

    return copyBufferSize;
}

int8_t ModbusHandler::process_FC6(ModbusHandler *modH )
{

    uint16_t add = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint8_t copyBufferSize;
    uint16_t val = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ] );

    modH->mRegs[ (ModbusRegisterIndex) add ] = val;

    // keep the same header
    modH->mBufferSize = to_underlying(PACKET_SIZE::RESPONSE);

    copyBufferSize = modH->mBufferSize + 2;
    sendTxBuffer(modH);

    return copyBufferSize;
}

int8_t ModbusHandler::process_FC15( ModbusHandler *modH )
{
    uint8_t currentBit, frameByte, bitsno;
    uint16_t currentRegister;
    uint8_t copyBufferSize;
    uint16_t currentCoil, coil;
    bool bTemp;

    // get the first and last coil from the message
    uint16_t startCoil = word( modH->mBuffer[ to_underlying(Message::ADD_HI) ], modH->mBuffer[ to_underlying(Message::ADD_LO) ] );
    uint16_t coilno = word( modH->mBuffer[ to_underlying(Message::NB_HI) ], modH->mBuffer[ to_underlying(Message::NB_LO) ] );


    // read each coil from the register map and put its value inside the outcoming message
    bitsno = 0;
    frameByte = 7;
    for (currentCoil = 0; currentCoil < coilno; currentCoil++)
    {

        coil = startCoil + currentCoil;
        currentRegister = (coil / 16);
        currentBit = (uint8_t) (coil % 16);

        bTemp = bitRead(
        			modH->mBuffer[ frameByte ],
                    bitsno );
        auto &reg = modH->mRegs[ (ModbusRegisterIndex) currentRegister ];
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
    modH->mBufferSize         = 6;
    copyBufferSize = modH->mBufferSize +2;
    sendTxBuffer(modH);
    return copyBufferSize;
}

int8_t ModbusHandler::process_FC16(ModbusHandler *modH )
{
    uint16_t startAdd = modH->mBuffer[ to_underlying(Message::ADD_HI) ] << 8 | modH->mBuffer[ to_underlying(Message::ADD_LO) ];
    uint16_t regsno = modH->mBuffer[ to_underlying(Message::NB_HI) ] << 8 | modH->mBuffer[ to_underlying(Message::NB_LO) ];
    uint8_t copyBufferSize;
    uint16_t i;
    uint16_t temp;

    // build header
    modH->mBuffer[ to_underlying(Message::NB_HI) ]   = 0;
    modH->mBuffer[ to_underlying(Message::NB_LO) ]   = (uint8_t) regsno; // answer is always 256 or less bytes
    modH->mBufferSize         = to_underlying(PACKET_SIZE::RESPONSE);

    // write registers
    for (i = 0; i < regsno; i++)
    {
        temp = word(
        		modH->mBuffer[ (to_underlying(Message::BYTE_CNT) + 1) + i * 2 ],
				modH->mBuffer[ (to_underlying(Message::BYTE_CNT) + 2) + i * 2 ]);
        if(modH->mRegs.find((ModbusRegisterIndex)(startAdd + i)) != modH->mRegs.end()) {
        	modH->mRegs[(ModbusRegisterIndex) (startAdd + i) ] = temp;
        }
    }
    copyBufferSize = modH->mBufferSize +2;
    sendTxBuffer(modH);

    return copyBufferSize;
}
}
