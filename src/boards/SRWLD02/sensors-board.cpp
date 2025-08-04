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
#include "fsmlist.h"
#include "board.h"

#define TEMPSENSOR_V25_TEMP           25.0

#define TEMPSENSOR_TYP_CAL1_V          (( int32_t)  760)        /*!< Internal temperature sensor, parameter V30 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define TEMPSENSOR_TYP_AVGSLOPE        (( int32_t) 2500)        /*!< Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
#define VREF_INT 1200                   //5.3.4 Embedded reference voltage
#define CALC_VDDA(vref) (roundf(4095.0 * VREF_INT/(vref)))
#define VDDA_MIN 2600
#define VDDA_MAX 3610

#define CHANNELS_PER_MUX_BLOCK 10
#define MUX_BLOCKS 2
#define REFERENCE_RES_HI 100000
#define REFERENCE_RES_LO 1000

#define DEFAULT_SAMPLE_PERIOD 1000 //ms

#define OVSF(bits) (std::pow(4, bits)) // Фактор оверсэмлинга (4^2) - используем белый шум потому 4
#define OVSD(bits) (std::pow(2, bits)) // Делитель для результатов оверсемплинга (деление на 4 эквивалентно сдвигу вправо на 2)
#define FOVS(fADCmax, oversampling_bits) = (fADCmax/(2.4*oversampling_bits)) //зменение частоты
#define OVSMV(new_resolution) ((1 << new_resolution) - 1) // Максимальное значение для нового разрешения

#define ADC_CALC_DATA_TO_VOLTAGE(VREFANALOG_VOLTAGE, ADC_DATA, ADC_RESOLUTION)  \
((ADC_DATA) * (VREFANALOG_VOLTAGE)                                   \
 / OVSMV(ADC_RESOLUTION)                                \
)

#define ADC_CALC_VREFANALOG_VOLTAGE(__VREFINT_ADC_DATA__,\
                                         __ADC_RESOLUTION__)                 \
(((uint32_t)(*VREFINT_CAL_ADDR) * VREFINT_CAL_VREF)                          \
 / __LL_ADC_CONVERT_DATA_RESOLUTION((__VREFINT_ADC_DATA__),                  \
                                    (__ADC_RESOLUTION__),                    \
                                    LL_ADC_RESOLUTION_12B)                   \
)

#define ADC_CALC_TEMPERATURE(__VREFANALOG_VOLTAGE__,\
                                  __TEMPSENSOR_ADC_DATA__,\
                                  __ADC_RESOLUTION__)\
((((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) != 0) ?        \
  (((( ((int32_t)((__LL_ADC_CONVERT_DATA_RESOLUTION((__TEMPSENSOR_ADC_DATA__),     \
                                                    (__ADC_RESOLUTION__),          \
                                                    LL_ADC_RESOLUTION_12B)         \
                   * (__VREFANALOG_VOLTAGE__))                                     \
                  / TEMPSENSOR_CAL_VREFANALOG)                                     \
        - (int32_t) *TEMPSENSOR_CAL1_ADDR)                                         \
     ) * (int32_t)(TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP)                    \
    ) / (int32_t)((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) \
   ) + TEMPSENSOR_CAL1_TEMP                                                        \
  )                                                                                \
  :                                                                                \
  ((int32_t)LL_ADC_TEMPERATURE_CALC_ERROR)                                         \
)

#define ADC_CALC_TEMPERATURE_TYP_PARAMS(__TEMPSENSOR_TYP_AVGSLOPE__,\
                                             __TEMPSENSOR_TYP_CALX_V__,\
                                             __TEMPSENSOR_CALX_TEMP__,\
                                             __VREFANALOG_VOLTAGE__,\
                                             __TEMPSENSOR_ADC_DATA__,\
                                             __ADC_RESOLUTION__)            \
(((((int32_t)((((__TEMPSENSOR_ADC_DATA__) * (__VREFANALOG_VOLTAGE__))       \
               / __LL_ADC_DIGITAL_SCALE(__ADC_RESOLUTION__))                \
              * 1000UL)                                                     \
    -                                                                       \
    (int32_t)(((__TEMPSENSOR_TYP_CALX_V__))                                 \
              * 1000UL)                                                     \
   )                                                                        \
  ) / (int32_t)(__TEMPSENSOR_TYP_AVGSLOPE__)                                \
 ) + (int32_t)(__TEMPSENSOR_CALX_TEMP__)                                    \
)


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
    { // канал 11 (13)
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

struct CalibrationChannels {
	CHANNEL_IDX lo;
	CHANNEL_IDX hi;
} gCalibrationChannels[MUX_BLOCKS] = {{CHANNEL_WL_LREF_MIN,	CHANNEL_WL_LREF_MAX}, {CHANNEL_WL_HREF_MIN,	CHANNEL_WL_HREF_MAX}};

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
		tinyfsm::Fsm<Channel>(),
		idx(id),
		type(type),
		limits(limits),
		onLimit(onLimit),
		config(config),
		shift(0),
		factor(1.0),
		calibration(UNCALIBRATED),
		connection(DISCONNECTED)
{
}

void Channel::Measure(Channel::ValueType &val){
	val = Measure();
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

Channel::ValueType Channel::Measure() {
    Channel::ValueType result;
    uint16_t total = 0;
    uint16_t resolution = 12; // Разрядность АЦП
    uint16_t oversampling_bits = 2;
    uint16_t oversampling_factor = (ADC_OVS_HARDWARE==ENABLE)? 1 : std::pow(4, oversampling_bits) - 1; // Фактор оверсэмлинга (4^bit) - используем белый шум потому 4
    uint16_t new_resolution = resolution + oversampling_bits; // Новое разрешение (14 бит)
    uint16_t oversampling_devider = std::pow(2, oversampling_bits);
    //Fovs = fADCmax/(2.4*oversampling_bits) //зменение частоты
    uint16_t max_value = (1 << new_resolution) - 1; // Максимальное значение для нового разрешения

    if(type ==Type::WL) {
		bool sensorsHalf = config.channel_en_pin == EN1;
		GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), sensorsHalf);
		GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), !sensorsHalf);
		osDelay(MUX_ENABLE_TIMEOUT);
		DataSampler::Instance().Mux().Select(config.channel_code);
		osDelay(MUX_SELECT_TIMEOUT);
		ToggleCurrentDirection(MUX_POL_SWITCH_TIMEOUT);
    }
    for (int i = 0; i < oversampling_factor; i++) {
    	if(type == Type::WL) {
			// Считывание значения АЦП
			if(DataSampler::Instance().MeasureMode() == DataSampler::AdcMode::SE) {

				total+= AdcReadChannel(const_cast<Adc_t*>(&AdcInP));
			}else {
				total+= AdcReadChannel(const_cast<Adc_t*>(&AdcInP)) - AdcReadChannel(const_cast<Adc_t*>(&AdcInN));
			}

    	}else{
    		total+= AdcReadChannel(type == Type::TS? &AdcTempSens : &AdcVref);
    	}
    }
#if(ADC_OVS_HARDWARE)
    uint16_t average = total/oversampling_factor; //calc just average as result already shifted by hardware
#else
    //Сдвиг вправо для получения нового разрешения (14 бит)
    uint16_t average = std::clamp(total /oversampling_devider, 0, (int)max_value);
#endif
    switch(type){
    case Channel::Type::VREF:{
    		//not needed - just placeholder
    	result =  ADC_CALC_DATA_TO_VOLTAGE(DataSampler::vdda_voltage, average, 14);
    	connection = CONNECTED;
    }
    break;
	case Channel::Type::WL:{
		result =  ADC_CALC_DATA_TO_VOLTAGE(DataSampler::vdda_voltage, average, 14);
		result = (result / CHANNEL_NOMINAL_CURRENT) *  1000;
		connection =result < CHANNEL_SHORTED_LIMIT? SHORTED: result < CHANNEL_DISCONNECTED_LIMIT?  CONNECTED : DISCONNECTED;
	}
	break;
	case Channel::Type::TS:{
		 if (((int32_t)*TEMPSENSOR_CAL2_ADDR - (int32_t)*TEMPSENSOR_CAL1_ADDR) != 0) {
			 result = ADC_CALC_TEMPERATURE(DataSampler::vdda_voltage,
					 	 	 	 	 	 	 	 average,
			                                     14);
		 }else{
			 result = ADC_CALC_TEMPERATURE_TYP_PARAMS(TEMPSENSOR_TYP_AVGSLOPE,
					 	 	 	 	 	 	 	 	 	   TEMPSENSOR_TYP_CAL1_V,
														   TEMPSENSOR_CAL1_TEMP,
														   DataSampler::vdda_voltage,
														   average,
														   14);
		 }
		connection = CONNECTED;
	}
	break;
	default:{
		}
    }
    Channel::ValueType value = (result  + shift) * factor;
    dispatch (MeasureEvent(idx, value));
    return value;
}


void Channel::react(const CalibrationStatusEvent& e){
	if((idx < e.channel) && (idx > e.channel - 10)){
		shift = e.shift;
		factor = e.factor;
		calibration = {CALIBRATED, std::chrono::system_clock::now()} ;
	}
}

void Channel::react(const MeasureEvent& e){
}


void SamplerTask(void * argument);
const osThreadAttr_t thread_attr = {
  .name = "DataSampler",
  .stack_size = 512 * 4,                            // Create the thread stack with a size of 1024 bytes
  .priority = (osPriority_t) osPriorityNormal
};
const osMemoryPoolAttr_t samples_attr = {
		.name = "samples"
};

float DataSampler::vdda_voltage = 0;

class Idle
: public DataSamplerFsm{
	void entry() override {
	  }
	void exit(void)  { };
};

class Sampling
: public DataSamplerFsm{

public:
	void entry() override {
	}
	void exit(void)  {
	};
};

class SelfTest
: public DataSamplerFsm {
	void entry() override {
		uint16_t total = 0;
		uint16_t resolution = 12; // Разрядность АЦП
	    uint16_t oversampling_bits = 2;
		uint16_t oversampling_factor = std::pow(4, oversampling_bits); // Фактор оверсэмлинга (4^2) - используем белый шум потому 4
		uint16_t new_resolution = resolution + oversampling_bits; // Новое разрешение (14 бит)
		uint16_t oversampling_devider = std::pow(2, oversampling_bits);
		uint16_t max_value = (1 << new_resolution) - 1; // Максимальное значение для нового разрешения
		//Initial vdda calibration
		DBG("ADC Sampler self test started\n");
		for(int i=0; i< oversampling_factor - 1; i++) {
			total += AdcReadChannel(&AdcVref);
		}
		total/=oversampling_devider;
		    //VDDA=4095 * 1.20 / ADC

		s->vdda_voltage = __LL_ADC_CALC_VREFANALOG_VOLTAGE(total,LL_ADC_RESOLUTION_12B);//mV
		if(s->vdda_voltage > VDDA_MIN && s->vdda_voltage < VDDA_MAX) {
			DBG("VDDA: %.3f\r\n", s->vdda_voltage);
			dispatch(SelfTestStatusEvent(Status::PASSED));
		}else {
			s->vdda_voltage = 0;
			DBG("vdda %0.3f- exceed limits\r\n", s->vdda_voltage);
			dispatch(SelfTestStatusEvent(Status::FAILED));
	   }
	  }
	void exit(void)  { };
};

class Calibrating
: public DataSamplerFsm{
	void entry() override {

		for(auto cb:gCalibrationChannels) {
			float shift;
			float factor;
			float voltage[MUX_BLOCKS];
			voltage[0] =  ADC_CALC_DATA_TO_VOLTAGE(s->vdda_voltage, s->mChannels[cb.lo].Measure(), 14);
			voltage[1] =  ADC_CALC_DATA_TO_VOLTAGE(s->vdda_voltage, s->mChannels[cb.hi].Measure(), 14);

			shift =   REFERENCE_RES_LO - (voltage[0] / (CHANNEL_NOMINAL_CURRENT / 1000.0));
			factor =  REFERENCE_RES_HI / (voltage[1] / (CHANNEL_NOMINAL_CURRENT / 1000.0)) - shift;
			if(shift < 200 && factor < 1.2 && factor > 0.8) {
				dispatch(CalibrationStatusEvent{PASSED, cb.lo, shift, factor});
			}else{
				dispatch(CalibrationStatusEvent{FAILED, cb.lo, shift, factor});
			}
		}
	}
	void exit(void)  { };
};

class CalibrationFailed
: public DataSamplerFsm{
	void entry() override {
	  }
	void exit(void)  { };
};

FSM_INITIAL_STATE(DataSamplerFsm, SelfTest)

DataSampler::DataSampler():
		fsm(),
		mSamplesMp(osMemoryPoolNew(2, sizeof(struct Samples), &samples_attr)),
		mSamplesMq(osMessageQueueNew(1, sizeof(Samples*), nullptr)),
		mMav(),
		mTs(),
		mSamplePeriod(DEFAULT_SAMPLE_PERIOD),
		mSamplePeriodReal(0),
		mAdcMode(AdcMode::SE),
		mSelfTest(UNKNOWN),
		mSelector({SCH0, SCH1, SCH2, SCH3}),
		mTaskHandle(osThreadNew(SamplerTask, this, &thread_attr))
{
	fsm.s = this;
	fsm.start();
}

DataSampler::~DataSampler(){
	DeInit();
	osMemoryPoolDelete(mSamplesMp);
	osMessageQueueDelete(mSamplesMq);
}

void DataSampler::DeInit(){

	AdcDeInit( &AdcInP );
	AdcDeInit( &AdcInN );
}

SamplerMode DataSampler::Mode() {
	return (BoardGetPowerSource() == EXT_POWER)? CONTINUOUS: ONESHOT;
}

void DataSampler::DoSamplerTask()
{
	DBG("ADC Sampler started\n");


	while( 1 ){
		Samples* sensorsData = static_cast<Samples*>(new(osMemoryPoolAlloc(mSamplesMp, osWaitForever)) Samples);
		//memset(sensorsData, 0, sizeof(Samples));
		if(sensorsData) {
			for (auto& channel: mChannels) {
				channel.Measure((*sensorsData)[channel.idx]);

			sensorsData->timestamp = std::chrono::system_clock::now();
			mSamplePeriodReal = std::chrono::duration_cast<std::chrono::milliseconds>(sensorsData->timestamp - mTs);
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
			osDelay(std::chrono::duration_cast<std::chrono::milliseconds>(mSamplePeriod  - (mSamplePeriodReal - mSamplePeriod)).count());
		}
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

void DataSamplerFsm::react(const InitStatusEvent &e)
{

}

void DataSamplerFsm::react(const SelfTestStatusEvent &e)
{
	s->mSelfTest  = e.result;
	if(e.result.status == PASSED){
		transit<Calibrating>();
	}else{
		transit<Idle>();
	}
}

void DataSamplerFsm::react(const  CalibrationStatusEvent &e)
{
	if(e.status == PASSED){
		for(auto ch:s->mChannels){
			ch.dispatch(e);
		}
		transit<Sampling>();
	}else{
		transit<Idle>();
	}
}

void DataSamplerFsm::react(const SampleEvent &e)
{

}
void DataSamplerFsm::react(const SampleDoneEvent &e)
{

}


