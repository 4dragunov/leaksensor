/*
 * modbus.h
 *
 *  Created on: Nov 26, 2024
 *      Author: Andrey Belyakov <andrei.belyxkov@simbirsoft.com>
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_


#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "uart.h"


#ifdef __cplusplus
#include "nonvol.h"

namespace ModBus {

#define T35  5              // Timer T35 period (in ticks) for end frame detection.
#define MAX_BUFFER  128	    // Maximum size for the communication buffer in bytes.
#define TIMEOUT_MODBUS 1000 // Timeout for master query (in ticks)
#define MAX_M_HANDLERS 2    //Maximum number of modbus handlers that can work concurrently
#define MAX_TELEGRAMS 2     //Max number of Telegrams in master queue


enum class PACKET_SIZE:uint8_t
{
    RESPONSE = 6,
    EXCEPTION = 3,
    CHECKSUM = 2
};

enum class Address:uint8_t
{
    BROADCAST = 0,  //!< broadcast mode -> buffer[ID] == 0
    NORMAL = 1,     //!< normal mode -> buffer[ID] > 0
};

enum class ModBusType:uint8_t {
	Slave= 3,
	Master = 4
};

/**
 * @enum MB_FC
 * @brief
 * Modbus function codes summary.
 * These are the implement function codes either for Master or for Slave.
 *
 * @see also fctsupported
 * @see also modbus_t
 */
enum class FunctionCode:uint8_t
{
    READ_COILS               = 1,	 /*!< FCT=1 -> read coils or digital outputs */
    READ_DISCRETE_INPUT      = 2,	 /*!< FCT=2 -> read digital inputs */
    READ_REGISTERS           = 3,	 /*!< FCT=3 -> read registers or analog outputs */
    READ_INPUT_REGISTER      = 4,	 /*!< FCT=4 -> read analog inputs */
    WRITE_COIL               = 5,	 /*!< FCT=5 -> write single coil or output */
    WRITE_REGISTER           = 6,	 /*!< FCT=6 -> write single register */
    WRITE_MULTIPLE_COILS     = 15, /*!< FCT=15 -> write multiple coils or outputs */
    WRITE_MULTIPLE_REGISTERS = 16	 /*!< FCT=16 -> write multiple registers */
};

enum class Message
{
    ID                             = 0, //!< ID field
    FUNC, //!< Function code position
    ADD_HI, //!< Address high byte
    ADD_LO, //!< Address low byte
    NB_HI, //!< Number of coils or registers high byte
    NB_LO, //!< Number of coils or registers low byte
    BYTE_CNT  //!< byte counter
};

enum class ComState:uint8_t
{
    IDLE                     = 0,
    WAITING                  = 1,

};

enum class Error:int8_t
{
	NONE = 0,
    NOT_MASTER                = -1,
    POLLING                   = -2,
    BUFF_OVERFLOW             = -3,
    BAD_CRC                   = -4,
    EXCEPTION                 = -5,
    BAD_SIZE                  = -6,
    BAD_ADDRESS               = -7,
    TIME_OUT		          = -8,
    BAD_SLAVE_ID		      = -9,
	BAD_TCP_ID		          = -10,
	OK_QUERY				  = -11
};

enum
{
    EXC_FUNC_CODE = 1,
    EXC_ADDR_RANGE = 2,
    EXC_REGS_QUANT = 3,
    EXC_EXECUTE = 4
};

typedef union {
	uint8_t  u8[4];
	uint16_t u16[2];
	uint32_t u32;
} byteFields ;




class ModbusRegister;
typedef void(*OnChanged)(ModbusRegister *reg);

enum class ModbusRegisterIndex{
	IDENT, //rw - nv
	BAUD_RATE,//rw - nv
	WORD_LEN,//rw - nv
	STOP_BITS,//rw - nv
	TEMP_SENSORS,//ro
	TEMP_1,//ro - mand
	TEMP_2,//ro - opt
	TEMP_3,//ro - opt
	TEMP_4,//ro - opt
	TEMP_5,//ro - opt
	TEMP_6,//ro - opt
	TEMP_7,//ro - opt
	TEMP_8,//ro - opt
	WL_SENSORS,//ro
	WL_1,//ro
	WL_2,//ro
	WL_3,//ro
	WL_4,//ro
	WL_5,//ro
	WL_6,//ro
	WL_7,//ro
	WL_8,//ro
	WL_9,//ro
	WL_10,//ro
	WL_12,//ro
	WL_13,//ro
	WL_14,//ro
	WL_15,//ro
	WL_16,//ro
	WL_17,//ro
	WL_18,//ro
	WL_19,//ro
	WL_20,//ro
	COUNT
};

class ModbusRegister  {

	uint16_t *reg;
	NvProperty<uint16_t> *prop;
	ModbusRegister* pair;
	OnChanged onChanged;
public:
	ModbusRegister():reg(),prop(),pair(),onChanged() {};
	ModbusRegister(uint16_t *reg, ModbusRegister *h = nullptr, NvProperty<uint16_t> *prop = nullptr, OnChanged changed = nullptr):
			reg(reg)
			,prop(prop)
			,pair(h)
			,onChanged(changed){

	};
	virtual ~ModbusRegister() = default;

	operator uint16_t& () {
		  return *reg;
	}

	// Write variable to RAM and through to backing store.
	uint16_t& operator = (const uint16_t& v)
	{
		*reg = v;
		update();
		return *reg;
	}
	void update(void) {
		if(onChanged)
			onChanged(this);
	};
};
typedef std::map<ModbusRegisterIndex, ModbusRegister> Registers;

/**
 * @class ModBusQuery
 * @brief
 * Master query structure:
 * This structure contains all the necessary fields to make the Master generate a Modbus query.
 * A Master may keep several of these structures and send them cyclically or
 * use them according to program needs.
 */
typedef struct Query
{
    uint8_t id;       /*!< Slave address between 1 and 247. 0 means broadcast */
    FunctionCode fct;   /*!< Function code: 1, 2, 3, 4, 5, 6, 15 or 16 */
    uint16_t regAdd;    /*!< Address of the first register to access at slave/s */
    uint16_t coilsNo;   /*!< Number of coils or registers to access */
    Registers reg;      /*!< Pointer to memory image in master */
    uint32_t *currentTask; /*!< Pointer to the task that will receive notifications from Modbus */
}Query_t;

template<typename T, uint8_t S>
	class RingBuffer {
	private:
		uint8_t mBuffer[S];
		uint8_t mBegin;
		uint8_t mEnd;
		uint8_t mAvailable;
		bool    mOverflow;
	public:
		RingBuffer():mBuffer(),mBegin(),mEnd(), mAvailable(), mOverflow(){Clear();};
		RingBuffer(const RingBuffer<T , S> & rb){
			mBegin = rb.mBegin;
			mEnd = rb.mEnd;
			mAvailable = rb.mAvailable;
			mOverflow = rb.mOverflow;
			memcpy(mBuffer, rb.mBuffer, sizeof(T) * S);
		}
		virtual ~RingBuffer() = default;

		//Function prototypes for RingBuffer
		void Add(uint8_t val)
		{

			mBuffer[mEnd] = val;
			mEnd = (mEnd + 1) % S;
			if (mAvailable == S)
			{
				mOverflow = true;
				mBegin = (mBegin + 1) % S;
			}
			else
			{
				mOverflow = false;
				mAvailable++;
			}

		}
		uint8_t GetAllBytes(uint8_t *buffer); // gets all the available bytes into buffer and return the number of bytes read
		uint8_t GetNBytes(uint8_t *buffer, uint8_t uNumber); // gets uNumber of bytes from ring buffer, returns the actual number of bytes read
		uint8_t CountBytes() const; // return the number of available bytes
		void Clear(); // flushes the ring buffer

		uint8_t* buffer(void) {return mBuffer;}

        bool  &overflow() {return mOverflow;}
        void overflow(bool s) {  mOverflow = s;}

        uint8_t& available() {return mAvailable;}
        void available(uint8_t s) {  mAvailable = s;}
};



/**
 * @class ModBus
 * @brief
 * Modbus handler structure
 * Contains all the variables required for Modbus daemon operation
 */

class ModbusHandler
{
	ModBusType mType;
	Uart_t *mUart; //HAL Serial Port handler
	NvProperty<uint8_t> mId; //!< 0=master, 1..247=slave number
	NvProperty<uint32_t> mBaudRate;
	NvProperty<uint8_t> mWordLen;
	NvProperty<uint8_t> mStopBits;
	NvProperty<uint8_t> mParity;
	Gpio_t * mDePin;  //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
	Error  mLastError;
	uint8_t mBuffer[MAX_BUFFER]; //Modbus buffer for communication
	uint8_t mBufferSize;
	uint8_t mLastRec;
	Registers mRegs;
	uint16_t mInCnt, mOutCnt, mErrCnt; //keep statistics of Modbus traffic
	uint16_t mTimeOut;
	uint8_t mDataRX;
	int8_t mState;

    Address mAddressMode; //!< 0=broadcast, 1..247=normal

	//FreeRTOS components

	//Queue Modbus Messqge
    osMessageQId mQueueMessageHandle;

	//Task
    osThreadId mTaskHandle;
	//Timer RX Modbus
	xTimerHandle mTimerT35;
	//Timer MasterTimeout
	xTimerHandle mTimerTimeout;
	//Semaphore for Modbus data
	osSemaphoreId mSpHandle;
	// RX ring buffer for USART
	RingBuffer<uint8_t,static_cast<unsigned char>(MAX_BUFFER)> mBufferRX;

	static void sendTxBuffer(ModbusHandler *mh);
	int16_t getRxBuffer();
	static uint8_t validateAnswer(ModbusHandler *mh);
	static void buildException( uint8_t exception, ModbusHandler *mh );
	static uint8_t validateRequest(ModbusHandler *mh);
	static uint16_t word(uint8_t H, uint8_t l);
	static uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);
	static void get_FC1(ModbusHandler *mh);
	static void get_FC3(ModbusHandler *mh);
	static int8_t process_FC1(ModbusHandler *mh);
	static int8_t process_FC3(ModbusHandler *mh);
	static int8_t process_FC5(ModbusHandler *mh);
	static int8_t process_FC6(ModbusHandler *mh);
	static int8_t process_FC15(ModbusHandler *mh);
	static int8_t process_FC16(ModbusHandler *mh);
	static void vTimerCallbackT35(TimerHandle_t *pxTimer);
	static void vTimerCallbackTimeout(TimerHandle_t *pxTimer);
	Error SendQuery(Query_t q);

public:
	typedef  RingBuffer<uint8_t,static_cast<unsigned char>(MAX_BUFFER)> BusBuffer;

	// Function prototypes
	ModbusHandler(Uart_t *uart, Gpio_t *dePin, ModBusType type, const uint8_t id, Registers &regs);
	virtual ~ModbusHandler(){};

	void Start();
	virtual void SetLine();

	void setTimeOut( uint16_t timeOut); //!<write communication watch-dog timer
	uint16_t getTimeOut(); //!<get communication watch-dog timer value
	bool getTimeOutState(); //!<get communication watch-dog timer state
	void Query(Query_t mq ); // put a query in the queue tail
	void QueryInject(Query_t mq); //put a query in the queue head
	static void SlaveTask(void *argument); //slave
	static void MasterTask(void *argument); //master
	static void MasterTask(const void *argument);
	static void SlaveTask(const void *argument);
	void * port() {return mUart;}
	osThreadId taskHandle() {return mTaskHandle;}
	xTimerHandle t35TimerHandle() {return mTimerT35;}
	BusBuffer& busBuffer() {return mBufferRX;}
	uint8_t &  dataRX() {return mDataRX;}
	Registers& registers() {return mRegs;}
};

extern uint8_t numberHandlers; //global variable to maintain the number of concurrent handlers
extern ModbusHandler *mHandlers[MAX_M_HANDLERS];


namespace Master {

class ModBusMaster:public ModbusHandler {
public:

	ModBusMaster(Uart_t *uart, Gpio_t *dePin, Registers &regs):ModbusHandler(uart, dePin, ModBusType::Master, 0, regs) {};
	virtual ~ModBusMaster()=default;
};
}//namespace Master

namespace Slave {

class ModBusSlave:public ModbusHandler {
public:

	ModBusSlave(Uart_t *uart, Gpio_t *dePin, const uint8_t id, Registers &regs):ModbusHandler(uart, dePin, ModBusType::Slave, id, regs) {};
	virtual ~ModBusSlave()=default;
};
} //namespace Slave

} //namespace ModBus

#endif // __cplusplus

#endif /* INC_MODBUS_H_ */
