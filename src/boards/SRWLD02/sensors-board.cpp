/*
 * sensors.cpp
 *
 *  Created on: Jan 18, 2025
 *      Author: Andrey
 */



#include <cassert>
#include <cfloat>
#include <climits>
#include <cmath>
#include <cmsis_os.h>

#include "stm32wlxx.h"
#include "utilities.h"
#include "gpio.h"
#include "sensors-board.h"
#include "stm32wlxx_ll_adc.h"
#include "board-config.h"
#include "adc.h"


#define TEMPSENSOR_V25_TEMP           25.0

#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)  //5.3.21 Temperature sensor characteristics
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)  //5.3.21 Temperature sensor characteristics
#define VREF_INT 1200                   //5.3.4 Embedded reference voltage
#define CALC_VDDA(vref) (roundf(4095.0 * VREF_INT/(vref)))
#define VDDA_MIN 2600
#define VDDA_MAX 3610




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

ChannelSelector<4> gChannelSelector({SCH0, SCH1, SCH2, SCH3});

//4 ref c channel (2 on each half) + ts and vref
const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + 4 + 2] = {

    {// канал 1
     .channel_en_pin = EN0,
	 .channel_code = 0
    },
    {// канал 2
     .channel_en_pin = EN0,
	 .channel_code = 1
    },
    {// канал 3
    .channel_en_pin = EN0,
	.channel_code = 2
    },
    { // канал 4
    .channel_en_pin = EN0,
	.channel_code = 3
    },
    {// канал 5
    .channel_en_pin = EN0,
	.channel_code = 4
    },
    {// канал 6
    .channel_en_pin = EN0,
	.channel_code = 5
    },
    {// канал 7
    .channel_en_pin = EN0,
	.channel_code = 6
    },
    {// канал 8
    .channel_en_pin = EN0,
	.channel_code = 7
    },
    {// канал 9
    .channel_en_pin = EN0,
	.channel_code = 8
    },
    {// канал 10
    .channel_en_pin = EN0,
	.channel_code = 9
    },
	{// канал 11 - lref min
	 .channel_en_pin = EN0,
	 .channel_code = 14
	},
	{// канал 12 - lref max
	 .channel_en_pin = EN0,
	 .channel_code = 15
	},
    { // канал 11
    .channel_en_pin = EN1,
	.channel_code = 0
    },
    { // канал 12
    .channel_en_pin = EN1,
	.channel_code = 1
    },
    { // канал 13
    .channel_en_pin = EN1,
	.channel_code = 2
    },
    { // канал 14
    .channel_en_pin = EN1,
	.channel_code = 3
    },
    {// канал 15
    .channel_en_pin = EN1,
	.channel_code = 4
    },
    {// канал 16
    .channel_en_pin = EN1,
	.channel_code = 5
    },
    { // канал 17
    .channel_en_pin = EN1,
	.channel_code = 6
    },
    {// канал 18
    .channel_en_pin = EN1,
	.channel_code = 7
    },
    { // канал 19
    .channel_en_pin = EN1,
	.channel_code = 8
    },
    {// канал 20
    .channel_en_pin = EN1,
	.channel_code = 9
    },
	{// канал href min
	 .channel_en_pin = EN1,
	 .channel_code = 14
	},
	{// канал href max
	 .channel_en_pin = EN1,
	 .channel_code = 15
	},
	{// канал 21 - ts
	 .channel_en_pin = NC,
	 .channel_code = 0,
	},
    {// канал 22
     .channel_en_pin = NC,
     .channel_code = 0,
    }
};


Channel::Limits default_wl_limits{0, 100, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_vref_limits{1000, 3500, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_ts_limits{0, 100, false, Channel::Units::TEMPERATURE,  1.0};

DataSampler::Channels DataSampler::mChannels ={

		Channel(CHANNEL_WL0, gChannelConfig[CHANNEL_WL0], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL1, gChannelConfig[CHANNEL_WL1], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL2, gChannelConfig[CHANNEL_WL2], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL3, gChannelConfig[CHANNEL_WL3], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL4, gChannelConfig[CHANNEL_WL4], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL5, gChannelConfig[CHANNEL_WL5], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL6, gChannelConfig[CHANNEL_WL6], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL7, gChannelConfig[CHANNEL_WL7], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL8, gChannelConfig[CHANNEL_WL8], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL9, gChannelConfig[CHANNEL_WL9], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_LREF_MIN, gChannelConfig[CHANNEL_WL_LREF_MIN], Channel::Type::VREF,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_LREF_MAX, gChannelConfig[CHANNEL_WL_LREF_MAX], Channel::Type::VREF,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL10, gChannelConfig[CHANNEL_WL10], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL11, gChannelConfig[CHANNEL_WL11], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL12, gChannelConfig[CHANNEL_WL12], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL13, gChannelConfig[CHANNEL_WL13], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL14, gChannelConfig[CHANNEL_WL14], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL15, gChannelConfig[CHANNEL_WL15], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL16, gChannelConfig[CHANNEL_WL16], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL17, gChannelConfig[CHANNEL_WL17], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL18, gChannelConfig[CHANNEL_WL18], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL19, gChannelConfig[CHANNEL_WL19], Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_HREF_MIN, gChannelConfig[CHANNEL_WL_HREF_MIN], Channel::Type::VREF,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_HREF_MAX, gChannelConfig[CHANNEL_WL_HREF_MAX], Channel::Type::VREF,   default_wl_limits,   DataSampler::OnChannelLimit),

		Channel(CHANNEL_TS,  gChannelConfig[CHANNEL_TS],  Channel::Type::TS,   default_ts_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_VREF, gChannelConfig[CHANNEL_VREF], Channel::Type::VREF, default_vref_limits, DataSampler::OnChannelLimit)
};

extern Adc_t  AdcVref;
extern Adc_t  AdcTempSens;
extern Gpio_t SensorsEn[2];
extern Gpio_t SensorsPolarity;
extern Adc_t  AdcInP;
extern Adc_t  AdcInN;

bool operator==(const Gpio_t& lhs, const Gpio_t& rhs)
{
    return ((lhs.port == rhs.port) && (lhs.pin == rhs.pin));
}

bool operator==(const Adc_t& lhs, const Adc_t& rhs)
{
    return (lhs.inst == rhs.inst) &&
    	   (lhs.channel == rhs.channel) &&
		   (lhs.AdcInput == rhs.AdcInput);
}

Channel::Channel(const CHANNEL_IDX id, const ChannelConfig &config, const Type type, const Limits limits, const OnLimit onLimit):
		idx(id),
		type(type),
		limits(limits),
		onLimit(onLimit),
		config(config)
{
}

void Channel::Measure(Channel::ValueType &val){
	val = Process();
	if((val > limits.hi || val) < limits.lo && onLimit)
		onLimit(this);
}

void Channel::ToggleCurrentDirection(uint16_t time_delay) {

		for (int i = 0; i < 5; i++) {
			GpioWrite(const_cast<Gpio_t*>(&SensorsPolarity), 1);

			osDelay(time_delay);
			GpioWrite(const_cast<Gpio_t*>(&SensorsPolarity), 0);
			osDelay(time_delay);
		}
		GpioWrite(const_cast<Gpio_t*>(&SensorsPolarity), 0);
		osDelay(time_delay);
}

Channel::ValueType Channel::Process() {
    const int num_measurements = 2;
    uint16_t adc_values[num_measurements];
    float sum = 0;
    uint16_t max_value = 0;
    uint16_t min_value = 0xFFFF; // �?нициализируем минимальное значение максимальным возможным
    if(config.channel_en_pin != NC) {
		bool sensorsHalf = config.channel_en_pin == EN1;
		GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), sensorsHalf);
		GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), !sensorsHalf);
    }
    gChannelSelector.Select(config.channel_code);

    for (int i = 0; i < num_measurements; i++) {

		ToggleCurrentDirection(30);
		osDelay(10);

        // Считывание значения АЦП
		if(ADC_MODE == SE) {
			adc_values[i] = AdcReadChannel(const_cast<Adc_t*>(&AdcInP));
		}else {
			adc_values[i] = AdcReadChannel(const_cast<Adc_t*>(&AdcInP)) - AdcReadChannel(const_cast<Adc_t*>(&AdcInN));
		}

        sum += adc_values[i];
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
        }
        if (adc_values[i] < min_value) {
            min_value = adc_values[i];
        }
    }

    uint16_t average = (uint16_t)round(sum / num_measurements);
    return average;
}


void SamplerTask(void * argument);
const osThreadAttr_t thread_attr = {
  .name = "DataSampler",
  .stack_size = 2048                            // Create the thread stack with a size of 1024 bytes
};
const osMemoryPoolAttr_t samples_attr = {
		.name = "samples"
};

float DataSampler::vdda_voltage = 0;

DataSampler::DataSampler():
		mSamplesMp(osMemoryPoolNew(2, sizeof(struct Samples), &samples_attr)),
		mSamplesMq(osMessageQueueNew(1, sizeof(Samples*), nullptr)),
		mMav(),
		mTs(),
		mSamplePeriod(),
		mTaskHandle(osThreadNew(SamplerTask, this, &thread_attr))
{
}

DataSampler::~DataSampler(){
	DeInit();
	osThreadTerminate(mTaskHandle);
	osMemoryPoolDelete(mSamplesMp);
	osMessageQueueDelete(mSamplesMq);
}

void DataSampler::DeInit(){

	AdcDeInit( &AdcInP );
	AdcDeInit( &AdcInN );
}

void DataSampler::DoSamplerTask()
{
	DBG("ADC Sampler started\n");
	float vref = 0;
	uint16_t vref_iteration = 0;
	float vref_tmp = 1500; // initial vref in adc units
	//Initial vdda calibration
	do {
		vref = vref_tmp;
		auto adc = AdcReadChannel(&AdcVref);
		vref_tmp =  (vref + adc)/2.0;
		vref_iteration++;
	}while((std::abs(vref - vref_tmp) >= 1) && vref_iteration < 30);
	assert(vref_iteration < 30);
	//VDDA=4095 * 1.20 / ADC
	DBG("VREF_IT:%i\r\n", vref_iteration);
    vdda_voltage = CALC_VDDA(vref);//mV
	assert(vdda_voltage > VDDA_MIN);
	assert(vdda_voltage < VDDA_MAX);

	while( 1 ){
		Samples* sensorsData = static_cast<Samples*>(new(osMemoryPoolAlloc(mSamplesMp, osWaitForever)) Samples);
		//memset(sensorsData, 0, sizeof(Samples));
		if(sensorsData) {
			for (auto& channel: mChannels) {
				channel.Measure((*sensorsData)[channel.idx]);
				//Initial channel scale
				switch(channel.chType()){
					case Channel::Type::WL:{
						auto voltage =  __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda_voltage, (*sensorsData)[channel.idx], LL_ADC_RESOLUTION_12B);
						(*sensorsData)[channel.idx] = roundf(100 * (voltage/vdda_voltage));
					}
					break;
					case Channel::Type::TS:{
						    /* Device with temperature sensor not calibrated in production:
						       use generic parameters */
							float temperature = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(INTERNAL_TEMPSENSOR_AVGSLOPE,
									INTERNAL_TEMPSENSOR_V25,
									TEMPSENSOR_V25_TEMP,
									vdda_voltage,
									(*sensorsData)[channel.idx],
									LL_ADC_RESOLUTION_12B);
							(*sensorsData)[channel.idx] = temperature * 10;

					}
					break;
					case Channel::Type::VREF:{
					//not needed - just placeholder
					}
					break;
					default:{
						DBG("Unsupported adc channel type\r\n");
						assert(0);
					}
				}
			}
			gettimeofday(&sensorsData->timestamp, 0);
			mSamplePeriod = sensorsData->timestamp - mTs;
			*sensorsData = mMav.Filter(sensorsData);
			vdda_voltage = (vdda_voltage + CALC_VDDA((*sensorsData)[CHANNEL_VREF]))/2.0;//mV;
			mTs = sensorsData->timestamp;
#ifdef DEBUG
			//std::cout << *sensorsData << std::endl;
			for(int i = 0; i < WL_CHANNEL_COUNT + 2; i++){
				DBG("s:%i:%i\n", i, sensorsData->data.raw[i]);
			}
#endif

#if 0
			osMemoryPoolFree(mSamplesMp, sensorsData);
#else
			osMessageQueuePut(mSamplesMq, &sensorsData, 0, osWaitForever);
#endif
		}
	}
}

void SamplerTask(void * argument){
	static_cast<DataSampler*>(argument)->DoSamplerTask();
}

void DataSampler::OnChannelLimit(const Channel *ch)
{
	DataSampler::Instance().DoOnChannelLimit(ch);
}

void DataSampler::DoOnChannelLimit(const Channel *ch)
{
	DBG("Channel %d reached limit\n", ch->idx);
}

void setSamplerate(struct timeval &tv)
{
	DBG("Sample rate changed\n");
}

struct timeval getSamplerate(void)
{
	DBG("Sample rate:\n");
}
