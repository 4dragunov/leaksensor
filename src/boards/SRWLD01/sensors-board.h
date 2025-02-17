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

#define WL_CHANNEL_COUNT 20
#define MAV_WINDOW 4

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
	CHANNEL_TS,
	CHANNEL_VREF,
	CHANNEL_COUNT
	} CHANNEL_IDX;

typedef struct {
	PinNames toggle_pin1;      // Первый пин для ToggleCurrentDirection
	PinNames toggle_pin2;      // Второй пин для ToggleCurrentDirection
	PinNames analog_pin;       // Вход АЦП
	uint32_t adc_channel;
	void* hadc;               // Указатель на обработчик АЦП
} ChannelConfig;

bool operator==(const Gpio_t& lhs, const Gpio_t& rhs);
bool operator==(const Adc_t& lhs, const Adc_t& rhs);

typedef struct ChannelPins{
	Gpio_t ptp;
	Gpio_t ntp;
	Adc_t ap;
	bool operator==(const ChannelPins &other){
		return ((ptp == other.ptp) &&
				(ntp == other.ntp) &&
				(ap == other.ap));
	}
} ChannelPins;

struct Channel{
	typedef int16_t ValueType;
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

	Channel(const CHANNEL_IDX id, ChannelPins &pins,  Type type, Limits limits, OnLimit onLimit = nullptr);
	virtual ~Channel() = default;

	void Measure(Channel::ValueType &val);
	Channel::ValueType Process(const ChannelPins& pins);
	void SetChannels(const ChannelPins& active_channel, uint8_t val);
	void ToggleCurrentDirection(const Gpio_t &pin1, const Gpio_t &pin2, uint16_t time_delay);
	Type chType(){return type;}
	operator Channel::ValueType&() {Channel::ValueType val; Measure(val); return val;}
    const CHANNEL_IDX idx;
	ChannelPins &pins;
	Type type;
	Limits limits;
	OnLimit     onLimit;

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
    struct timeval timestamp;
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


class DataSampler {
	friend void SamplerTask(void* argument);
public:
	 typedef std::array<Channel, WL_CHANNEL_COUNT + 2> Channels;

	 static DataSampler &Instance() {
		 static DataSampler instance;
		 return instance;
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
protected:
	 static Channels mChannels;
	 osMemoryPoolId_t mSamplesMp;
	 osMessageQueueId_t mSamplesMq;
	 MAV<Samples, MAV_WINDOW> mMav;
	 struct timeval mTs;
	 struct timeval  mSamplePeriod;
	 osThreadId_t mTaskHandle;

	 DataSampler();
	 virtual ~DataSampler();
	 void DoSamplerTask();
};
extern const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + 2];
extern  ChannelPins gChannelsPins[WL_CHANNEL_COUNT + 2];
#endif


