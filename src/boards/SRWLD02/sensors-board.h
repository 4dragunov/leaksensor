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

#include "tinyfsm.hpp"
//#include "fsmlist.hpp"

#define WL_CHANNEL_COUNT 20
#define CAL_CHANNEL_COUNT 4
#define VREF_TS_CHANNEL_COUNT 2
#define MAV_WINDOW 4

#define CHANNEL_SHORTED_LIMIT 100 //ohm
#define CHANNEL_DISCONNECTED_LIMIT 120000 //ohm
#define CHANNEL_NOMINAL_CURRENT 20 //uA

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
	CHANNEL_TS,
	CHANNEL_VREF,
	CHANNEL_COUNT
	} CHANNEL_IDX;

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
   SelfTestResult(const Status st):
	   status(st),
	   time(std::chrono::system_clock::now()){}
}  SelfTestResult;

//Connection state
typedef enum  {
	DISCONNECTED,
	CONNECTED,
	SHORTED
} ChannelConnectionStatus;

typedef enum  {
	UNCALIBRATED,
	CALIBRATED
} ChannelCalibrationStatus;

typedef struct {
	ChannelCalibrationStatus status;
	std::chrono::time_point<std::chrono::system_clock> time;
} ChannelCalibration;

struct ChannelEvent: tinyfsm::Event {
	CHANNEL_IDX channel;
	ChannelEvent(CHANNEL_IDX ch):
		tinyfsm::Event(),
		channel(ch) {}
};


struct CalibrationStatusEvent: ChannelEvent {
  const Status status;
  const float &shift;
  const float &factor;
  CalibrationStatusEvent(const Status st, const CHANNEL_IDX &ch, const float &sh, const float &fa):
	  ChannelEvent(ch),
	  status(st),
	  shift(sh),
	  factor(fa)
 {

 }
};

struct MeasurementEvent       : ChannelEvent { };


#define Q2F(q) (q / 32768)
#define F2Q(f) (f * 32768)

class Channel: public tinyfsm::Fsm<Channel>{
public:
	typedef float ValueType;
	typedef void (*OnLimit)(const Channel *ch);
	enum class Units:uint8_t {
		VOLTAGE,
		TEMPERATURE
	};
	enum class Type:uint8_t{
		WL 	= 0, //water lavel
		TS, 	 //thermal sensor
		VREF, 	 //vref
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
		MeasureEvent(const CHANNEL_IDX &ch, const Channel::ValueType &v):ChannelEvent(ch), value(v) {}
	};

	Channel(const CHANNEL_IDX id, const ChannelConfig &config, const  Type type, const  Limits limits, const OnLimit onLimit = nullptr);
	virtual ~Channel() = default;

	virtual void entry(void) {};
	void exit(void)  { };
	virtual void react(const CalibrationStatusEvent& e);
	virtual void react(const MeasureEvent& e);

	void Measure(Channel::ValueType &val);
	Channel::ValueType Measure();
	void ToggleCurrentDirection(uint16_t time_delay);
	Type chType(){return type;}
	operator Channel::ValueType() {Channel::ValueType val; Measure(val); return val;}
    const CHANNEL_IDX idx;
	const Type type;
	const Limits limits;
	const OnLimit     onLimit;
	const ChannelConfig &config;
	Channel::ValueType value;
	float shift;
	float factor;
	ChannelCalibration calibration;
	ChannelConnectionStatus  connection;
};



typedef struct Samples{
	union Data{
		std::array<Channel::ValueType, WL_CHANNEL_COUNT + 1 + 1> raw;
		struct {
			std::array<Channel::ValueType, WL_CHANNEL_COUNT>  wl;
			Channel::ValueType Ts;
			Channel::ValueType Vref;
		} ch;
	}data;
	std::chrono::time_point<std::chrono::system_clock> timestamp;
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

struct InitStatusEvent       : tinyfsm::Event { };
struct SelfTestStatusEvent      : tinyfsm::Event {
	SelfTestResult result;
	SelfTestStatusEvent(const Status &st):tinyfsm::Event(), result(st){}
};

struct SampleEvent  : tinyfsm::Event { };
struct SampleDoneEvent : tinyfsm::Event {};


template<unsigned int Bits=4>
class ChannelSelector {
	std::array<Gpio_t,Bits> mPins;
	std::array<PinNames, Bits> mPinNames;
	uint8_t mSelected;
public:
	ChannelSelector(const std::array<PinNames, Bits>& names):
		mPins{},
		mPinNames{names},
		mSelected(0)
	{
		for(uint8_t bit = 0; bit < Bits; bit++){
			GpioInit( &mPins[bit], mPinNames[bit], PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		}
	}
	virtual ~ChannelSelector(void){
		for(uint8_t bit = 0; bit < Bits; bit++){
			GpioInit( &mPins[bit], mPinNames[bit], PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
		}
	}
	bool Select(uint8_t channel){

		if(channel && channel < (1 << Bits)){
			for(uint8_t bit = 0; bit < Bits; bit++){
				GpioWrite(&mPins[bit], ((channel - 1) >> bit) & 0x01);
			}
			mSelected = channel;
			return true;
		}else
			return false;
	}
	operator int(){return mSelected;}
	//0 means none selected
	uint8_t Current(){return mSelected;}
};

class DataSampler;
class DataSamplerFsm: public tinyfsm::Fsm<DataSamplerFsm> {
public:
	DataSamplerFsm():tinyfsm::Fsm<DataSamplerFsm>(),s(), mTaskHandle(){}
	virtual ~DataSamplerFsm(){}

	 virtual void entry(void) {};
	 virtual void exit(void)  {};

	 void react(tinyfsm::Event &e) {};
	 virtual void react(const InitStatusEvent &e);
	 virtual void react(const SelfTestStatusEvent &e);
	 virtual void react(const CalibrationStatusEvent &e);
	 virtual void react(const SampleEvent &e);
	 virtual void react(const SampleDoneEvent &e);
	 DataSampler* s;
	 osThreadId_t mTaskHandle;
};

class DataSampler {
	friend void SamplerTask(void* argument);
	friend class DataSamplerFsm;
	friend class Calibrating;
	friend class SelfTest;
public:
	 typedef std::array<Channel, WL_CHANNEL_COUNT + 2 + 4> Channels;
	 typedef ChannelSelector<4> Multiplexer;
     typedef enum {SE,DIFF}AdcMode;


	 void react(tinyfsm::Event &e) {fsm.react(e);};
	 virtual void react(const InitStatusEvent &e) {fsm.react(e);};
	 virtual void react(const SelfTestStatusEvent &e){fsm.react(e);};
	 virtual void react(const CalibrationStatusEvent &e){fsm.react(e);};
	 virtual void react(const SampleEvent &e){fsm.react(e);};
	 virtual void react(const SampleDoneEvent &e){fsm.react(e);};

	 static DataSampler &Instance() {
		 static DataSampler instance;
		 return  instance;
	 }
	 void DeInit();

	 static void OnChannelLimit(const Channel *ch);
	 void DoOnChannelLimit(const Channel *ch);
	 Channels& channels(){return mChannels;}
	 Channel &operator[](const size_t idx) {
	 		return mChannels[idx];
	 	}
	 void setSamplerate(struct timeval &tv);
	 struct timeval getSamplerate(void);
	 osMemoryPoolId_t Pool() { return mSamplesMp;}
	 osMessageQueueId_t Queue() {return mSamplesMq;}
	 static float vdda_voltage;
	 SamplerMode Mode();
	 AdcMode     MeasureMode(){return mAdcMode;}
	 Multiplexer& Mux() { return mSelector;};
protected:

	 static Channels mChannels;
	 DataSamplerFsm fsm;
	 osMemoryPoolId_t mSamplesMp;
	 osMessageQueueId_t mSamplesMq;
	 MAV<Samples, MAV_WINDOW> mMav;
	 std::chrono::time_point<std::chrono::system_clock> mTs;
	 std::chrono::milliseconds  mSamplePeriod;
	 std::chrono::milliseconds  mSamplePeriodReal;
	 AdcMode      mAdcMode;
	 SelfTestResult  mSelfTest;
	 Multiplexer mSelector;
	 osThreadId_t mTaskHandle;
	 DataSampler();
	 virtual ~DataSampler();
	 void DoSamplerTask();
};
extern const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + 4 + 2];
#endif


