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
#include <map>
#include <vector>
#include <functional>
#include <variant>
#include <ostream>
#include "utilities.h"
#include "nonvol.h"

namespace ModBus {

#define COMMA ,

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

// helper type for the visitor #4
template<class... Ts>
struct overloaded : Ts... { using Ts::operator()...; };
// explicit deduction guide (not needed as of C++20)
template<class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;



class Register {

	template <typename Var1, typename Var2> struct variant_flat;

	template <typename ... Ts1, typename ... Ts2>
	struct variant_flat<std::variant<Ts1...>, std::variant<Ts2...>>
	{
	    using type = std::variant<Ts1..., Ts2...>;
	};
public:
	friend std::ostream& operator<<(std::ostream&, const ModBus::Register&);
	enum class Index: uint8_t {
		BEGIN = 0,
		END
	};

	enum class Access{
				RO,
				RW,
				WO
	};
	typedef void (*OnChanged)(const Register *reg);
	typedef void (*OnAccessError)(const Register *reg);

	template<typename T>
	struct RefValue{
		T& value;
		T  min;
		T  max;
	 RefValue(T&value, T min, T max):value(value),min(min),max(max){}
	 operator const T& () const {
		return value;
	}

	T& operator = (const T& v) {
		value = std::clamp(v, min, max);
		return value;
	}
	};
    typedef std::variant<
    		RefValue<bool>,
			RefValue<uint8_t>,
			RefValue<uint16_t>,
			RefValue<uint32_t>,
			RefValue<int8_t>,
			RefValue<int16_t>,
			RefValue<int32_t>
    > RefValueTypes;
    using nvb_ref  = std::reference_wrapper<NvProperty<bool>>;
    using nv8_ref  = std::reference_wrapper<NvProperty<uint8_t>>;
    using nv16_ref = std::reference_wrapper<NvProperty<uint16_t>>;
    using nv32_ref = std::reference_wrapper<NvProperty<uint32_t>>;
    using nv64_ref = std::reference_wrapper<NvProperty<uint64_t>>;
    typedef std::variant<
    		nvb_ref,
			nv8_ref,
			nv16_ref,
			nv32_ref,
			nv64_ref
    > NvValueTypes;
    using VariantType_all = variant_flat<RefValueTypes, NvValueTypes>::type;
    typedef std::vector<VariantType_all>  ValuesType;

	typedef std::function<uint16_t (const Register::ValuesType &vs)> GetterType;
	typedef std::function<void(Register::ValuesType &vs, const uint16_t value)> SetterType;
	const SetterType defaultSetter = [](Register::ValuesType &nvp, const uint16_t value){
		if(std::holds_alternative<Register::nvb_ref>(nvp[0])) {
			std::get<Register::nvb_ref>(nvp[0]).get() = value;
		} else
		if(std::holds_alternative<Register::nv8_ref>(nvp[0])) {
			std::get<Register::nv8_ref>(nvp[0]).get() = value;
		}else
		if(std::holds_alternative<Register::nv16_ref>(nvp[0])) {
			std::get<Register::nv16_ref>(nvp[0]).get() = value;
		}else
		if(std::holds_alternative<Register::nv32_ref>(nvp[0])) {
			std::get<Register::nv32_ref>(nvp[0]).get() = value;
		}else
		if(std::holds_alternative<Register::RefValue<bool>>(nvp[0])) {
			std::get<Register::RefValue<bool>>(nvp[0]) = value;
		} else
		if(std::holds_alternative<Register::RefValue<uint8_t>>(nvp[0])) {
			std::get<Register::RefValue<uint8_t>>(nvp[0]) = value;
		}else
		if(std::holds_alternative<Register::RefValue<uint16_t>>(nvp[0])) {
			std::get<Register::RefValue<uint16_t>>(nvp[0]) = value;
		}else
		if(std::holds_alternative<Register::RefValue<uint32_t>>(nvp[0])) {
			std::get<Register::RefValue<uint32_t>>(nvp[0]) = value;
		}
		return value;
	};
	const GetterType defaultGetter = [](const Register::ValuesType &nvp)->uint16_t{
		if(std::holds_alternative<Register::nvb_ref>(nvp[0])) {
			return std::get<Register::nvb_ref>(nvp[0]).get();
		} else
		if(std::holds_alternative<Register::nv8_ref>(nvp[0])) {
			return std::get<Register::nv8_ref>(nvp[0]).get();
		}else
		if(std::holds_alternative<Register::nv16_ref>(nvp[0])) {
			return std::get<Register::nv16_ref>(nvp[0]).get();
		}else
		if(std::holds_alternative<Register::nv32_ref>(nvp[0])) {
			return std::get<Register::nv32_ref>(nvp[0]).get();
		}else
		if(std::holds_alternative<Register::RefValue<bool>>(nvp[0])) {
			return std::get<Register::RefValue<bool>>(nvp[0]);
		} else
		if(std::holds_alternative<Register::RefValue<uint8_t>>(nvp[0])) {
			return std::get<Register::RefValue<uint8_t>>(nvp[0]);
		}else
		if(std::holds_alternative<Register::RefValue<uint16_t>>(nvp[0])) {
			return std::get<Register::RefValue<uint16_t>>(nvp[0]);
		}else
		if(std::holds_alternative<Register::RefValue<uint32_t>>(nvp[0])) {
			return std::get<Register::RefValue<uint32_t>>(nvp[0]);
		}
		return 0;
	};
	const OnChanged defaultOnChanged = [](const Register *reg){};
	const OnAccessError defaultOnAccessError = [](const Register *reg){};
	Register();
	Register(Register::Index idx, Register::ValuesType values,
			Register::Access acc = Register::Access::RW,
			Register::GetterType getter = [](const Register::ValuesType &vs)->uint16_t{return 0;},
			Register::SetterType setter = [](Register::ValuesType &vs, const uint16_t value){},
			Register::OnChanged changed = nullptr,
			Register::OnAccessError error = nullptr);


	virtual ~Register() = default;

	virtual operator const uint16_t& ();
	virtual const uint16_t& operator = (const uint16_t& value);
	virtual const uint16_t& operator &= (const uint16_t& value);
	virtual const uint16_t& operator |= (const uint16_t& value);
	const Access  acces;

protected:

private:
	Register::Index mIdx;
	ValuesType mValues;
	GetterType mGetter;
	SetterType mSetter;
	OnChanged  mOnChanged;
	OnAccessError mOnAccessError;
};

enum class Index: std::underlying_type_t<ModBus::Register::Index> {
		BEGIN =  static_cast<std::underlying_type_t<ModBus::Register::Index>>(ModBus::Register::Index::END),
		IDENT = BEGIN,
		BAUD_RATE_AND_WORD_LEN,
		STOP_BITS_AND_PARITY,//rw - nv
		END
};
#define REGISTER(idx, val, acc, set, get) static_cast<ModBus::Register::Index>(idx), ModBus::Register(static_cast<ModBus::Register::Index>(idx), {val}, acc, set, get)

std::ostream& operator<<(std::ostream&, const ModBus::Register&);

using Registers = std::map<Register::Index, Register>;

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
class Modbus
{
	friend void ModBus_RxCpltCallback(Uart_t *huart);
	ModBusType mType;
	Uart_t *mUart; //HAL Serial Port handler
	NvProperty<uint8_t> mId;        //!< 0=master, 1..247=slave number
	NvProperty<uint8_t> mBaudRate; // as 115200/baud
	NvProperty<uint8_t> mWordLen;
	NvProperty<uint8_t> mStopBits;
	NvProperty<uint8_t> mParity;
	Gpio_t * mDePin;  //!< flow control pin: 0=USB or RS-232 mode, >1=RS-485 mode
	Error  mLastError;
	uint8_t mBuffer[MAX_BUFFER]; //Modbus buffer for communication
	uint8_t mBufferSize;
	uint8_t mLastRec;
	Registers mRegs;
	typedef Registers::key_type Index_type;
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
    osTimerId mTimerT35;
	//Timer MasterTimeout
    osTimerId mTimerTimeout;
	//Semaphore for Modbus data
	osSemaphoreId mSpHandle;
	// RX ring buffer for USART
	RingBuffer<uint8_t,static_cast<unsigned char>(MAX_BUFFER)> mBufferRX;

	void sendTxBuffer();
	int16_t getRxBuffer();
	uint8_t validateAnswer();
	void buildException( uint8_t exception);
	uint8_t validateRequest();
	static uint16_t word(uint8_t H, uint8_t l);
	static uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);
	void get_FC1();
	void get_FC3();
	int8_t process_FC1();
	int8_t process_FC3();
	int8_t process_FC5();
	int8_t process_FC6();
	int8_t process_FC15();
	int8_t process_FC16();
	static void vTimerCallbackT35(void const * arg);
	static void vTimerCallbackTimeout(void const * arg);
	Error SendQuery(Query_t q);

public:
	typedef  RingBuffer<uint8_t,static_cast<unsigned char>(MAX_BUFFER)> BusBuffer;

	// Function prototypes
	Modbus(Uart_t *uart, Gpio_t *dePin, ModBusType type, const uint8_t id, Registers &regs);
	virtual ~Modbus(){};

	void Start();
	virtual void SetLine(const uint32_t baudrate = 9600, const WordLength_t wordLength=UART_8_BIT, const StopBits_t stopBits = UART_1_STOP_BIT, const Parity_t parity = NO_PARITY);

	void setTimeOut( uint16_t timeOut); //!<write communication watch-dog timer
	uint16_t getTimeOut(); //!<get communication watch-dog timer value
	bool getTimeOutState(); //!<get communication watch-dog timer state
	void Query(Query_t mq ); // put a query in the queue tail
	void QueryInject(Query_t mq); //put a query in the queue head
	void DoSlaveTask(); //slave
	void DoMasterTask(); //master
	static void MasterTask(const void *argument);
	static void SlaveTask(const void *argument);
	Uart_t* port() {return mUart;}
	osThreadId taskHandle() {return mTaskHandle;}
	osTimerId  &t35TimerHandle() {return mTimerT35;}
	BusBuffer& busBuffer() {return mBufferRX;}
	uint8_t &  dataRX() {return mDataRX;}
	Registers& registers() {return mRegs;}
	Register& operator[](const size_t idx) {return mRegs[static_cast<Register::Index>(idx)];}
};

extern volatile uint8_t numberHandlers; //global variable to maintain the number of concurrent handlers
extern volatile Modbus *mHandlers[MAX_M_HANDLERS];


namespace Master {

class ModBusMaster:public Modbus {
public:

	ModBusMaster(Uart_t *uart, Gpio_t *dePin, Registers &regs):Modbus(uart, dePin, ModBusType::Master, 0, regs) {

	}
	virtual ~ModBusMaster()=default;
};
}//namespace Master

namespace Slave {

class ModBusSlave:public Modbus {
public:

	ModBusSlave(Uart_t *uart, Gpio_t *dePin, const uint8_t id, Registers &regs):Modbus(uart, dePin, ModBusType::Slave, id, regs) {
	}
	virtual ~ModBusSlave()=default;
};
} //namespace Slave

} //namespace ModBus

extern void ModBus_IrqNotify(Uart_t *uart, UartNotifyId_t type);
extern void ModBus_TxCpltCallback(Uart_t *huart);
extern void ModBus_RxCpltCallback(Uart_t *huart);

#endif // __cplusplus

#endif /* INC_MODBUS_H_ */
