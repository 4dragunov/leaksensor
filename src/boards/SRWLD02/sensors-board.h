/*
 * adc.h
 *
 *  Created on: Sep 1, 2024
 *      Author: Andrey Belyakov <andrei.belyakov@simbirsoft.com>
 */

#pragma once

#ifdef __cplusplus
#include <ostream>
#include <array>
#include <stdint.h>
#include <stdbool.h>
#include <cstddef>
#include <cassert>
#include <adc.h>
#include "mav.h"
#include "arm_math.h"
#include <cmsis_os.h>
#include <ctime>
#include <chrono>
#include "arm_math.h"
#include "freertos_mpool.h"
#include "tinyfsm.hpp"
//#include "fsmlist.hpp"

#define WL_CHANNEL_COUNT 20
#define WL_CHANNEL_HALF_COUNT 10
#define CAL_CHANNEL_COUNT 4
#define VREF_VBAT_AND_TEMP_CHANNEL_COUNT 3
#define MAV_WINDOW 4

#define CHANNEL_SHORTED_LIMIT 100 //ohm
#define CHANNEL_DISCONNECTED_LIMIT 120000 //ohm
#define CHANNEL_NOMINAL_CURRENT 20 //uA (microAmpere)
#define CALIBRATION_VALID_TIME 3600//sec
#define CALIBRATION_REPEAT 30//sec
#define SELF_TEST_REPEAT   30//sec

typedef enum {
	CHANNEL_WL0 = 0,
	CHANNEL_WL1,
	CHANNEL_WL2,
	CHANNEL_WL3,
	CHANNEL_WL4,
	CHANNEL_WL5,
	CHANNEL_WL6,
	CHANNEL_WL7,
	CHANNEL_WL8,
	CHANNEL_WL9,
	CHANNEL_WL_LREF_MIN,
	CHANNEL_WL_LREF_MAX,
	CHANNEL_WL10,
	CHANNEL_WL11,
	CHANNEL_WL12,
	CHANNEL_WL13,
	CHANNEL_WL14,
	CHANNEL_WL15,
	CHANNEL_WL16,
	CHANNEL_WL17,
	CHANNEL_WL18,
	CHANNEL_WL19,
	CHANNEL_WL_HREF_MIN,
	CHANNEL_WL_HREF_MAX,
	CHANNEL_MUX_COUNT,
	CHANNEL_TS = CHANNEL_MUX_COUNT,
	CHANNEL_VREF,
	CHANNEL_VBAT,
	CHANNEL_COUNT
	} ChannelIdx;

typedef struct {
	PinNames channel_en_pin;      // Первый пин для ToggleCurrentDirection
	uint8_t channel_code;
} ChannelConfig;

bool operator==(const Gpio_t& lhs, const Gpio_t& rhs);
bool operator==(const Adc_t& lhs, const Adc_t& rhs);

//self test state
typedef enum  {
	UNKNOWN,
	PASSED,
	FAILED
} Status;

typedef struct SelfTestResult{
   Status status;
   std::chrono::time_point<std::chrono::system_clock> time;
   SelfTestResult():status(UNKNOWN),time(std::chrono::system_clock::now()){}
   SelfTestResult(const Status &st):
	   status(st),
	   time(std::chrono::system_clock::now()){}
}  SelfTestResult;


/*---------------------------------------------------------------------------------------*/

class Channel;
struct ChannelEvent: tinyfsm::Event {
	const Channel& channel;
	ChannelEvent(const Channel& ch):
		tinyfsm::Event(),
		channel(ch) {}
};

struct CalibrationStatusEvent: ChannelEvent {
  const Status status;
  const float &shift;
  const float &factor;
  const float &error;
  CalibrationStatusEvent(const Status st, const Channel &ch, const float &sh, const float &fa, const float& er):
	  ChannelEvent(const_cast<Channel&>(ch)),
	  status(st),
	  shift(sh),
	  factor(fa),
	  error(er)
 {

 }
};

struct CalibrationEndedEvent: tinyfsm::Event {};
struct MeasurementEvent       : ChannelEvent { };
struct SelfTestStatusEvent      : tinyfsm::Event {
	SelfTestResult result;
	SelfTestStatusEvent(const Status &st):tinyfsm::Event(), result(st){}
};

struct SampleEvent  : tinyfsm::Event { };
struct SampleDoneEvent : tinyfsm::Event {};

#define Q2F(q) (q / 32768)
#define F2Q(f) (f * 32768)

class Channel{
public:
	typedef float ValueType;
	typedef void (*OnLimit)(const Channel *ch);
	enum class Units:uint8_t {
		VOLTAGE,
		TEMPERATURE
	};
	enum class Type:uint8_t{
		WL 	= 0, //water lavel
		CAL,
		TS, 	 //thermal sensor
		VREF, 	 //vref
		VBAT,    // rtc battery
		COUNT
	};

	typedef struct Limits{
		Channel::ValueType lo;
		Channel::ValueType hi;
		bool   valid;
		Units  units;
	    float    div;
	}Limits;

	struct MeasureEvent: ChannelEvent {
		const Channel::ValueType &value;
		MeasureEvent(const Channel &ch, const Channel::ValueType &v):ChannelEvent(ch), value(v) {}
	};
	//Connection state

	class Connection {
	public:
		typedef enum {
			DISCONNECTED,
			CONNECTED,
			SHORTED
		}Status;

		Connection():time(std::chrono::system_clock::now()),state(DISCONNECTED){};

		void react(tinyfsm::Event const &) { };
		virtual void react(Channel::MeasureEvent const &e);

		std::chrono::time_point<std::chrono::system_clock> time;
		Status state;
	};

	class Calibration {
	public:
		Calibration(bool requred):time(),shift(0),factor(1),status(UNKNOWN),requred(requred){}
		virtual ~Calibration(){}

		void react(tinyfsm::Event const &) { };
		virtual void react(CalibrationStatusEvent const &);
		std::chrono::seconds elapsedTime();
		std::chrono::time_point<std::chrono::system_clock> time;
		float shift;
		float factor;
		float error;
		Status status;
		bool requred;
	};
	Channel(const ChannelIdx id, const ChannelConfig &config, const  Type type, const  Limits limits, const OnLimit onLimit = nullptr);
	virtual ~Channel() = default;


	virtual void react(const tinyfsm::Event&e){};
	virtual bool react(const CalibrationStatusEvent& e);
	virtual void react(const MeasureEvent& e);

	void Measure(Channel::ValueType &val, size_t averaging = 1);
	Channel::ValueType Measure(size_t averaging = 1);
	void ToggleCurrentDirection(uint16_t time_delay);
	Type chType(){return type;}
	operator Channel::ValueType() {Channel::ValueType val; Measure(val); return val;}

    const ChannelIdx idx;
	const Type type;
	const Limits limits;
	const OnLimit     onLimit;
	const ChannelConfig &config;
	Channel::ValueType value;
	Calibration calibration;
	Connection  connection;
};

typedef struct Samples{
	union Data{
		std::array<Channel::ValueType, ChannelIdx::CHANNEL_COUNT> raw;
		struct {
			std::array<Channel::ValueType, WL_CHANNEL_HALF_COUNT>  wl1;
			Channel::ValueType refmin1;
			Channel::ValueType refmax1;
			std::array<Channel::ValueType, WL_CHANNEL_HALF_COUNT>  wl2;
			Channel::ValueType refmin2;
			Channel::ValueType refmax2;
			Channel::ValueType Ts;
			Channel::ValueType Vref;
			Channel::ValueType Vbat;
		} ch;
	}data;
	std::chrono::time_point<std::chrono::system_clock> timestamp;
	std::chrono::milliseconds duration;

	Samples():data(),timestamp(std::chrono::system_clock::now()),duration(){}
    virtual ~Samples() = default;

    std::chrono::milliseconds& sampled(){
    	if(duration.count() == 0)
    		duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timestamp) ;
    	return duration;
    }

    friend  std::ostream& operator<<(std::ostream& os, const Samples& s) {
    	os << "ts s:" << s.timestamp << std::endl;
    	auto size = s.data.raw.size();
    	for(unsigned i = 0; i <  size -1; i++){
    		os << i << ":" << s.data.raw[i] << std::endl;
    	}
    	return os;
    }

	inline bool operator==(const Samples& other)
    {
		assert(data.raw.size() == other.data.raw.size());
		for (uint8_t i=0; i< data.raw.size(); i++){
			if(data.raw[i]!=other.data.raw[i])
				return false;
		}
        return true;
    }
    inline bool operator!=(const Samples& lhs) {
    	return !(*this==lhs);
    }
    Samples& operator+=(const Samples& b) {
    	assert(data.raw.size() == b.data.raw.size());
		for (uint8_t i=0; i< data.raw.size(); i++){
			data.raw[i]+=b.data.raw[i];
		}
		return *this;
	}
    Samples& operator+=(volatile Samples& b) {
    	assert(data.raw.size() == const_cast<Samples&>(b).data.raw.size());
		for (uint8_t i=0; i< data.raw.size(); i++){
			data.raw[i]+=const_cast<Samples&>(b).data.raw[i];
		}
		return *this;
	}

    Samples& operator-=(const Samples& b) {
		for (uint8_t i=0; i< data.raw.size(); i++){
			data.raw[i]-=b.data.raw[i];
		}
		return *this;
	}

    Samples operator+(const Samples& b) const
	{
    	Samples a(*this);
		for (uint8_t i=0; i< data.raw.size(); i++){
					a.data.raw[i]+=b.data.raw[i];
		}
		return a;
	}

    Samples operator-(const Samples& b) const
	{
    	Samples a(*this);
		for (uint8_t i=0; i<  data.raw.size(); i++){
					a.data.raw[i]-=b.data.raw[i];
		}
		return a;
	}
    Samples operator/(const uint8_t &v) const
	{
    	Samples c;
		for (uint8_t i=0; i< data.raw.size(); i++){
			c.data.raw[i] = data.raw[i]/v;
		}
		return c;
	}
    Samples operator*(const Channel::Limits *limits) const
	{
    	Samples c;
		assert(limits);
		for (uint8_t i=0; i< data.raw.size(); i++) {
			c.data.raw[i] = data.raw[i] * limits[i].div;
		}
		return c;
	}
    Channel::ValueType &operator[](const size_t idx) {
		return data.raw[idx];
	}
}Samples;

typedef enum {
	ONESHOT,
	CONTINUOUS
}SamplerMode;

template<unsigned int Bits=4>
class Multiplexer {
	std::array<Gpio_t, Bits> mPins;
	std::array<PinNames, Bits> mPinNames;
	const ChannelConfig* mConfig;
public:
	Multiplexer(const std::array<PinNames, Bits>& names);
	virtual ~Multiplexer(void);
	void Select(const ChannelConfig &config);
	uint8_t CurrentChannel() const;
	ChannelConfig& CurrentConfig(){return *(ChannelConfig*)mConfig;}
	void Sleep();
	operator int(){return CurrentChannel();}
};

class DataSampler:public tinyfsm::Fsm<DataSampler> {
	friend class Fsm;
	friend void SamplerTask(void* argument);
//	friend class DataSamplerFsm;
	friend class Calibrating;
	friend class SelfTest;
	friend class Channel;
public:
	 typedef std::array<Channel, ChannelIdx::CHANNEL_COUNT> Channels;
	 typedef Multiplexer<4> Multiplexer;
     virtual void entry(void) {};
     virtual void exit(void)  {};

	 void react(tinyfsm::Event &e);
	 virtual void react(const SelfTestStatusEvent &e);
	 virtual void react(const CalibrationStatusEvent &e);
	 virtual void react(const CalibrationEndedEvent &e);
	 virtual void react(const SampleEvent &e);
	 virtual void react(const SampleDoneEvent &e);

	 static DataSampler &Instance() {
		 assert(tinyfsm::Fsm<DataSampler>::current_state_ptr);
		 return  *tinyfsm::Fsm<DataSampler>::current_state_ptr;
	 }
	 void DeInit();

	 static void OnChannelLimit(const Channel *ch);
	 void DoOnChannelLimit(const Channel *ch);
	 Channels& channels(){return mChannels;}
	 Channel &operator[](const size_t idx) {
	 		return mChannels[idx];
	 	}
	 void setSamplePeriod(std::chrono::milliseconds  samplePeriod) {mSamplePeriod = samplePeriod;}
	 std::chrono::milliseconds& getSamplePeriod(void) {return mSamplePeriodReal;};
	 osMemoryPoolId_t Pool() { return mSamplesMp;}
	 osMessageQueueId_t Queue() {return mSamplesMq;}
	 static uint16_t vdda_voltage;
	 SamplerMode Mode();
	 static AdcMode  MeasureMode(){return mAdcMode;}
	 Multiplexer& Mux() { return mMultiplexer;};
	 void Calibrate();
	 void StartSelfTest();
protected:
	 DataSampler();
	 virtual ~DataSampler();
	 static Channels mChannels;
	 //DataSamplerFsm fsm;
	 std::chrono::time_point<std::chrono::system_clock> mCalibrated;
	 std::chrono::time_point<std::chrono::system_clock> mTested;
	 MemPool_t* mSamplesMp;
	 osMessageQueueId_t mSamplesMq;
	 MAV<Samples, MAV_WINDOW> mMav;
	 std::chrono::time_point<std::chrono::system_clock> mTs;
	 std::chrono::milliseconds  mSamplePeriod;
	 std::chrono::milliseconds  mSamplePeriodReal;
	 static AdcMode      mAdcMode;
	 static Multiplexer mMultiplexer;
	 SelfTestResult  mSelfTest;
	 osThreadId_t mTaskHandle;

	 void DoSamplerTask();
};
extern const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + CAL_CHANNEL_COUNT + VREF_VBAT_AND_TEMP_CHANNEL_COUNT];
#endif //__cplusplus
