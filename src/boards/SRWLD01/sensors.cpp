/*
 * sensors.cpp
 *
 *  Created on: Jan 18, 2025
 *      Author: Andrey
 */



#include <cassert>
#include <math.h>
#include <cmsis_os.h>
#include "stm32f1xx.h"
#include "utilities.h"
#include "gpio.h"
#include <sys/time.h>
#include "sensors.h"


const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT] = {

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
    }
};

ChannelPins gChannelsPins[WL_CHANNEL_COUNT];

Samples DataSampler::mSamples = {};

Channel::Limits default_wl_limits{0, 100, false, Channel::Units::VOLTAGE,  1.0};
//Channel::Limits default_vref_limits{0, 100, false, Channel::Units::VOLTAGE,  1.0};
//Channel::Limits default_ts_limits{0, 100, false, Channel::Units::TEMPERATURE,  1.0};

DataSampler::Channels DataSampler::mChannels ={

		Channel(0, gChannelsPins[0],  DataSampler::mSamples.data.raw[0],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(1, gChannelsPins[1],  DataSampler::mSamples.data.raw[1],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(2, gChannelsPins[2],  DataSampler::mSamples.data.raw[2],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(3, gChannelsPins[3],  DataSampler::mSamples.data.raw[3],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(4, gChannelsPins[4],  DataSampler::mSamples.data.raw[4],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(5, gChannelsPins[5],  DataSampler::mSamples.data.raw[5],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(6, gChannelsPins[6],  DataSampler::mSamples.data.raw[6],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(7, gChannelsPins[7],  DataSampler::mSamples.data.raw[7],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(8, gChannelsPins[8],  DataSampler::mSamples.data.raw[8],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(9, gChannelsPins[9],  DataSampler::mSamples.data.raw[9],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(10, gChannelsPins[10],  DataSampler::mSamples.data.raw[10],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(11, gChannelsPins[11],  DataSampler::mSamples.data.raw[11],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(12, gChannelsPins[12],  DataSampler::mSamples.data.raw[12],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(13, gChannelsPins[13],  DataSampler::mSamples.data.raw[13],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(14, gChannelsPins[14],  DataSampler::mSamples.data.raw[14],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(15, gChannelsPins[15],  DataSampler::mSamples.data.raw[15],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(16, gChannelsPins[16],  DataSampler::mSamples.data.raw[16],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(17, gChannelsPins[17],  DataSampler::mSamples.data.raw[17],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(18, gChannelsPins[18],  DataSampler::mSamples.data.raw[18],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit),
		Channel(19, gChannelsPins[19],  DataSampler::mSamples.data.raw[19],  Channel::Type::WL, default_wl_limits, DataSampler::OnChannelLimit)
};


osThreadDef(samplerTask, DataSampler::SamplerTask, osPriorityNormal, 0, 256);
osMailQDef (samplesMq, 1, Samples);  // Declare mail queue

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

Channel::Channel(const uint8_t id, ChannelPins &pins, uint16_t &val, Type type, Limits limits, OnLimit onLimit):
		id(id),
		pins(pins),
		val(val),
		type(type),
		limits(limits),
		onLimit(onLimit)
{
}

uint16_t&  Channel::Measure(){
	val = Process(pins);
	if((val > limits.hi || val) < limits.lo && onLimit)
		onLimit(this);
	return val;
}

void Channel::SetChannels(const ChannelPins& active_channel, uint8_t val) {
    // Перебираем все каналы и подаем высокий логический уровень на неактивные
    for (int i = 0; i < WL_CHANNEL_COUNT; i++) {
        if (gChannelsPins[i] != active_channel) {
            GpioWrite(&gChannelsPins[i].ptp, val);
            GpioWrite(&gChannelsPins[i].ntp, val);
        }
    }
}


void Channel::ToggleCurrentDirection(const Gpio_t &pin1, const Gpio_t &pin2, uint16_t time_delay) {
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

uint16_t Channel::Process(const ChannelPins& channel) {
    const int num_measurements = 2;
    uint16_t adc_values[num_measurements];
    float sum = 0;
    uint16_t max_value = 0;
    uint16_t min_value = 0xFFFF; // �?нициализируем минимальное значение максимальным возможным

    // Устанавливаем высокий логический уровень на неиспользуемые каналы
    SetChannels(channel, 1);


    for (int i = 0; i < num_measurements; i++) {
        ToggleCurrentDirection(channel.ptp, channel.ntp, 30);
        GpioWrite(&gChannelsPins[i].ntp, 1); // подача высокого уровня перед АЦП
       // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); //led on
        osDelay(10);

        // Считывание значения АЦП

        adc_values[i] = AdcReadChannel(const_cast<Adc_t*>(&channel.ap));

        sum += adc_values[i];
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
        }
        if (adc_values[i] < min_value) {
            min_value = adc_values[i];
        }

        GpioWrite(const_cast<Gpio_t*>(&channel.ptp), 0); // подача низкого уровня перед АЦП
        osDelay(10);
    }

    uint16_t average = (uint16_t)round(sum / num_measurements);
    // Сброс логического уровня на неиспользуемых каналах после измерения
    SetChannels(channel, 0);
    return average;
}

DataSampler::DataSampler():
		mTaskHandle(osThreadCreate(osThread(samplerTask), this)),
		mSamplesMq(osMailCreate(osMailQ(samplesMq), mTaskHandle)),
		mMav(),
		mTs()
{
}

DataSampler::~DataSampler(){
	DeInit();
	osThreadTerminate(mTaskHandle);
}

osMailQId&  DataSampler::Init(){
return mSamplesMq;
}

void DataSampler::DeInit(){
	for(int i = 0; i < WL_CHANNEL_COUNT; i++)
	   AdcDeInit( &gChannelsPins[i].ap );
}

void DataSampler::SamplerTask(void const * argument)
{
	DataSampler *sampler = (DataSampler*)argument;
	DBG("ADC Sampler started\n");
	while( 1 ){
		Samples* sensorsData = (Samples*)osMailAlloc(*sampler, osWaitForever);
		if(sensorsData) {
			for (int i = 0; i < WL_CHANNEL_COUNT; i++) {
				mChannels[i].Measure();
			}
			*sensorsData = sampler->mMav.Filter(&sampler->samples());

			gettimeofday(&sensorsData->timestamp,0);
			sampler->mTs = sensorsData->timestamp;
			osMailPut(*sampler, (void*)sensorsData);
		}
	}
}

void DataSampler::OnChannelLimit(const Channel *ch)
{
	DataSampler::Instance().DoOnChannelLimit(ch);
}

void DataSampler::DoOnChannelLimit(const Channel *ch)
{
	DBG("Channel %d reached limit %i\n", ch->id, ch->val);
}

void setSamplerate(struct timeval &tv)
{
	DBG("Sample rate changed\n");
}

struct timeval getSamplerate(void)
{
	DBG("Sample rate:\n");
}

