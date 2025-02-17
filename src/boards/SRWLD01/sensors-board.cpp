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

#include "stm32f1xx.h"
#include "utilities.h"
#include "gpio.h"
#include "sensors-board.h"
#include "stm32f1xx_ll_adc.h"

#define TEMPSENSOR_CAL1_ADDR               ((uint16_t*) (0x1FF800FAU))
#define TEMPSENSOR_CAL2_ADDR               ((uint16_t*) (0x1FF800FEU))

#define TEMPSENSOR_V25_TEMP           25.0

#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)  //5.3.21 Temperature sensor characteristics
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)  //5.3.21 Temperature sensor characteristics
#define VREF_INT 1200                   //5.3.4 Embedded reference voltage
#define CALC_VDDA(vref) (roundf(4095.0 * VREF_INT/(vref)))
#define VDDA_MIN 2600
#define VDDA_MAX 3610




const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + 2] = {

    {// канал 1
     .toggle_pin1 = PE_12,
     .toggle_pin2 = PE_1,
	 .analog_pin =  PB_1,
	 .adc_channel = ADC_CHANNEL_9,
	 .hadc = (void*)ADC1
    },
    {// канал 2
     .toggle_pin1 = PE_13,
     .toggle_pin2 = PE_0,
	 .analog_pin =  PB_0,
	 .adc_channel = ADC_CHANNEL_8,
	 .hadc = (void*)ADC1
    },
    {// канал 3
     .toggle_pin1 = PE_14,
     .toggle_pin2 =	PB_9,
	 .analog_pin =  PC_5,
	 .adc_channel = ADC_CHANNEL_15,
	 .hadc = (void*)ADC1
    },
    { // канал 4
     .toggle_pin1 = PE_15,
	 .toggle_pin2 = PB_8,
	 .analog_pin =  PC_4,
	 .adc_channel = ADC_CHANNEL_14,
	 .hadc = (void*)ADC1
    },
    {// канал 5
     .toggle_pin1 =PB_10,
     .toggle_pin2 = PG_14,
	 .analog_pin =  PA_7,
	 .adc_channel = ADC_CHANNEL_7,
	 .hadc = (void*)ADC1
    },
    {// канал 6
     .toggle_pin1 =PD_9,
     .toggle_pin2 = PG_13,
	 .analog_pin =  PA_6,
	 .adc_channel = ADC_CHANNEL_6,
	 .hadc = (void*)ADC1
    },
    {// канал 7
     .toggle_pin1 =PD_10,
     .toggle_pin2 = PG_12,
	 .analog_pin =  PA_5,
	 .adc_channel = ADC_CHANNEL_5,
	 .hadc = (void*)ADC1
    },
    {// канал 8
     .toggle_pin1 = PD_11,
     .toggle_pin2 = PG_11,
	 .analog_pin =  PA_4,
	 .adc_channel = ADC_CHANNEL_4,
	 .hadc = (void*)ADC1
    },
    {// канал 9
     .toggle_pin1 = PD_12,
     .toggle_pin2 = PG_10,
	 .analog_pin =  PA_3,
	 .adc_channel = ADC_CHANNEL_3,
	 .hadc = (void*)ADC1
    },
    {// канал 10
     .toggle_pin1 = PD_13,
     .toggle_pin2 = PG_9,
	 .analog_pin =  PA_2,
	 .adc_channel = ADC_CHANNEL_2,
	 .hadc = (void*)ADC1
    },
    { // канал 11
     .toggle_pin1 = PD_14,
     .toggle_pin2 = PD_3,
	 .analog_pin =  PA_1,
	 .adc_channel = ADC_CHANNEL_1,
	 .hadc =(void*)ADC1
    },
    { // канал 12
     .toggle_pin1 = PD_15,
     .toggle_pin2 = PD_2,
	 .analog_pin =  PC_3,
	 .adc_channel = ADC_CHANNEL_13,
	 .hadc = (void*)ADC1
    },
    { // канал 13
     .toggle_pin1 = PG_2,
     .toggle_pin2 = PD_1,
	 .analog_pin =  PC_2,
	 .adc_channel = ADC_CHANNEL_12,
	 .hadc =(void*)ADC1
    },
    { // канал 14
     .toggle_pin1 = PG_3,
     .toggle_pin2 = PD_0,
     .analog_pin =  PC_1,
	 .adc_channel = ADC_CHANNEL_11,
	 .hadc = (void*)ADC1
    },
    {// канал 15
     .toggle_pin1 = PG_4,
     .toggle_pin2 = PC_11,
	 .analog_pin =  PC_0,
	 .adc_channel = ADC_CHANNEL_10,
	 .hadc = (void*)ADC1
    },
    {// канал 16
     .toggle_pin1 = PG_5,
     .toggle_pin2 = PC_10,
	 .analog_pin =  PF_10,
	 .adc_channel = ADC_CHANNEL_8,
	 .hadc = (void*)ADC3
    },
    { // канал 17
     .toggle_pin1 = PG_6,
     .toggle_pin2 = PA_12,
	 .analog_pin =  PF_9,
	 .adc_channel = ADC_CHANNEL_7,
	 .hadc = (void*)ADC3
    },
    {// канал 18
     .toggle_pin1 = PG_7,
     .toggle_pin2 = PA_11,
	 .analog_pin =  PF_8,
	 .adc_channel = ADC_CHANNEL_6,
	 .hadc = (void*)ADC3
    },
    { // канал 19
     .toggle_pin1 = PC_6,
     .toggle_pin2 = PC_9,
	 .analog_pin =  PF_7,
	 .adc_channel = ADC_CHANNEL_5,
	 .hadc = (void*)ADC3
    },
    {// канал 20
     .toggle_pin1 = PC_7,
     .toggle_pin2 = PC_8,
	 .analog_pin =  PF_6,
	 .adc_channel = ADC_CHANNEL_4,
	 .hadc = (void*)ADC3
    },
	{// канал 21 - ts
	 .toggle_pin1 = NC,
	 .toggle_pin2 = NC,
	 .analog_pin =  NC,
	 .adc_channel = ADC_CHANNEL_TEMPSENSOR,
	 .hadc = (void*)ADC1
	},
    {// канал 22
     .toggle_pin1 = NC,
     .toggle_pin2 = NC,
	 .analog_pin =  NC,
   	 .adc_channel = ADC_CHANNEL_VREFINT,
   	 .hadc = (void*)ADC1
    }
};

ChannelPins gChannelsPins[WL_CHANNEL_COUNT + 2];

Channel::Limits default_wl_limits{0, 100, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_vref_limits{1000, 3500, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_ts_limits{0, 100, false, Channel::Units::TEMPERATURE,  1.0};

DataSampler::Channels DataSampler::mChannels ={

		Channel(CHANNEL_WL0,  gChannelsPins[0],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL1,  gChannelsPins[1],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL2,  gChannelsPins[2],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL3,  gChannelsPins[3],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL4,  gChannelsPins[4],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL5,  gChannelsPins[5],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL6,  gChannelsPins[6],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL7,  gChannelsPins[7],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL8,  gChannelsPins[8],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL9,  gChannelsPins[9],   Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL10, gChannelsPins[10],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL11, gChannelsPins[11],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL12, gChannelsPins[12],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL13, gChannelsPins[13],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL14, gChannelsPins[14],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL15, gChannelsPins[15],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL16, gChannelsPins[16],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL17, gChannelsPins[17],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL18, gChannelsPins[18],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL19, gChannelsPins[19],  Channel::Type::WL,   default_wl_limits,   DataSampler::OnChannelLimit),

		Channel(CHANNEL_TS, gChannelsPins[20],  Channel::Type::TS,   default_ts_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_VREF, gChannelsPins[21],  Channel::Type::VREF, default_vref_limits, DataSampler::OnChannelLimit)
};

extern Adc_t  AdcVref;
extern Adc_t  AdcTempSens;

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

Channel::Channel(const CHANNEL_IDX id, ChannelPins &pins, Type type, Limits limits, OnLimit onLimit):
		idx(id),
		pins(pins),
		type(type),
		limits(limits),
		onLimit(onLimit)
{
}

void Channel::Measure(Channel::ValueType &val){
	val = Process(pins);
	if((val > limits.hi || val) < limits.lo && onLimit)
		onLimit(this);
}

void Channel::SetChannels(const ChannelPins& active_channel, uint8_t val) {
    // Перебираем все каналы и подаем высокий логический уровень на неактивные
    for (int i = 0; i < WL_CHANNEL_COUNT; i++) {
        if (gChannelsPins[i] != active_channel) {
        	if(gChannelsPins[i].ptp.pin != NC && gChannelsPins[i].ntp.pin != NC) {
				GpioWrite(&gChannelsPins[i].ptp, val);
				GpioWrite(&gChannelsPins[i].ntp, val);
        	}
        }
    }
}

void Channel::ToggleCurrentDirection(const Gpio_t &pin1, const Gpio_t &pin2, uint16_t time_delay) {
	if (pin1.pin != NC && pin2.pin != NC) {
		for (int i = 0; i < 5; i++) {
			GpioWrite(const_cast<Gpio_t*>(&pin1), 1);
			GpioWrite(const_cast<Gpio_t*>(&pin2), 0);
			osDelay(time_delay);
			GpioWrite(const_cast<Gpio_t*>(&pin1), 0);
			GpioWrite(const_cast<Gpio_t*>(&pin2), 1);
			osDelay(time_delay);
		}
		GpioWrite(const_cast<Gpio_t*>(&pin1), 0);
		GpioWrite(const_cast<Gpio_t*>(&pin2), 0);
		osDelay(time_delay);
	}
}

Channel::ValueType Channel::Process(const ChannelPins& channel) {
    const int num_measurements = 2;
    uint16_t adc_values[num_measurements];
    float sum = 0;
    uint16_t max_value = 0;
    uint16_t min_value = 0xFFFF; // �?нициализируем минимальное значение максимальным возможным

    // Устанавливаем высокий логический уровень на неиспользуемые каналы
    SetChannels(channel, 1);


    for (int i = 0; i < num_measurements; i++) {
    	if (channel.ptp.pin != NC && channel.ntp.pin != NC) {
			ToggleCurrentDirection(channel.ptp, channel.ntp, 30);
			GpioWrite(&gChannelsPins[i].ntp, 1); // подача высокого уровня перед АЦП
		   // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); //led on
			osDelay(10);
    	}
        // Считывание значения АЦП

        adc_values[i] = AdcReadChannel(const_cast<Adc_t*>(&channel.ap));

        sum += adc_values[i];
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
        }
        if (adc_values[i] < min_value) {
            min_value = adc_values[i];
        }
        if (channel.ptp.pin != NC) {
			GpioWrite(const_cast<Gpio_t*>(&channel.ptp), 0); // подача низкого уровня перед АЦП
			osDelay(10);
        }
    }

    uint16_t average = (uint16_t)round(sum / num_measurements);
    // Сброс логического уровня на неиспользуемых каналах после измерения
    SetChannels(channel, 0);
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
		mSamplesMp(osMemoryPoolNew(1, sizeof(Samples), &samples_attr)),
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
	for(int i = 0; i < WL_CHANNEL_COUNT; i++)
	   AdcDeInit( &gChannelsPins[i].ap );
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
		Samples* sensorsData = static_cast<Samples*>(osMemoryPoolAlloc(mSamplesMp, osWaitForever));
		memset(sensorsData, 0, sizeof(Samples));
		if(sensorsData) {
			for (auto& channel: mChannels) {
				channel.Measure((*sensorsData)[channel.idx]);
				//Initial channel scale
				switch(channel.chType()){
					case Channel::Type::WL:{
						auto voltage =  __LL_ADC_CALC_DATA_TO_VOLTAGE(vdda_voltage, (*sensorsData)[channel.idx], LL_ADC_RESOLUTION_12B);
						(*sensorsData)[channel.idx] = voltage;
					}
					break;
					case Channel::Type::TS:{

#ifdef __LL_ADC_CALC_TEMPERATURE
							(*sensorsData)[channel.id] = __LL_ADC_CALC_TEMPERATURE(vdda_voltage,
																			(*sensorsData)[channel.id],
																			LL_ADC_RESOLUTION_12B) * 100.0;
#else
						    /* Device with temperature sensor not calibrated in production:
						       use generic parameters */
							auto temperature = __LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(INTERNAL_TEMPSENSOR_AVGSLOPE,
									INTERNAL_TEMPSENSOR_V25,
									TEMPSENSOR_V25_TEMP,
									vdda_voltage,
									(*sensorsData)[channel.idx],
									LL_ADC_RESOLUTION_12b);
							(*sensorsData)[channel.idx] = temperature;
#endif
					}
					break;
					case Channel::Type::VREF:{
					}
					break;
					default:{
						DBG("Unsupported adc channel type\r\n");
						assert(0);
					}
				}
			}
			*sensorsData = mMav.Filter(sensorsData);
			gettimeofday(&sensorsData->timestamp,0);
			mSamplePeriod = sensorsData->timestamp - mTs;
			vdda_voltage = (vdda_voltage + CALC_VDDA((*sensorsData)[CHANNEL_VREF]))/2.0;//mV;
			mTs = sensorsData->timestamp;
			osMessageQueuePut(mSamplesMq, &sensorsData, 0, osWaitForever);
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
