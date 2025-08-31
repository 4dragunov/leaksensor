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

#define DEBUG

#define DEBUG_SAMPLING 0

#define VDDA_MIN 2600
#define VDDA_MAX 3610

#define CHANNELS_PER_MUX_BLOCK 10
#define CALIBRATION_AVERAGE 20
#define SAMPLING_AVERAGE 2
#define MUX_BLOCKS 2
#define REFERENCE_RES_HI 100000.0
#define REFERENCE_RES_LO 1000.0
#define REFERENCE_RES_TOLERANCE 1.0
#define MAX_ERROR 5
#define MAX_SHIFT_ERROR 1000

#define DEFAULT_SAMPLE_PERIOD 1000 //ms

float calc_temperature(uint16_t __VREFANALOG_VOLTAGE__,int16_t  __TEMPSENSOR_ADC_DATA__, uint8_t __ADC_RESOLUTION__){

	/*
	 *  Temperature = ((TS_ADC_DATA - TS_CAL1) * (TS_CAL2_TEMP - TS_CAL1_TEMP)) / (TS_CAL2 - TS_CAL1) + TS_CAL1_TEMP
	 */
    float TS_CAL1 = FB2B((int32_t) *TEMPSENSOR_CAL1_ADDR, 12, __ADC_RESOLUTION__);
	float TS_CAL2 = FB2B((int32_t) *TEMPSENSOR_CAL2_ADDR, 12, __ADC_RESOLUTION__);
	float TEMPSENSOR_CAL_TEMP_DIFF = (TEMPSENSOR_CAL2_TEMP - TEMPSENSOR_CAL1_TEMP);
	float TEMPSENSOR_CAL_DIFF = (TS_CAL2 - TS_CAL1);
	float TS_ADC_DATA = (__TEMPSENSOR_ADC_DATA__ * __VREFANALOG_VOLTAGE__) / TEMPSENSOR_CAL_VREFANALOG;
	if(TS_CAL2 - TS_CAL1)
		return ((TEMPSENSOR_CAL_TEMP_DIFF * (TS_ADC_DATA - TS_CAL1)) / TEMPSENSOR_CAL_DIFF) + TEMPSENSOR_CAL1_TEMP;
	else
		return	((int32_t)LL_ADC_TEMPERATURE_CALC_ERROR);
}

//4 ref c channel (2 on each half) + ts and vref
const ChannelConfig gChannelConfig[WL_CHANNEL_COUNT + CAL_CHANNEL_COUNT + VREF_VBAT_AND_TEMP_CHANNEL_COUNT] = {

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
	/*------------------------------------------*/
	{// канал lref min
	 .channel_en_pin = EN0,
	 .channel_code = 14
	},
	{// канал lref max
	 .channel_en_pin = EN0,
	 .channel_code = 15
	},
	/*------------------------------------------*/
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
	/*------------------------------------------*/
	{// канал href min
	 .channel_en_pin = EN1,
	 .channel_code = 14
	},
	{// канал href max
	 .channel_en_pin = EN1,
	 .channel_code = 15
	},
	/*------------------------------------------*/
	{// канал ts
	 .channel_en_pin = NC,
	 .channel_code = 0
	},
    {// канал vref
     .channel_en_pin = NC,
     .channel_code = 0
    },
	{// канал vbat
     .channel_en_pin = NC,
     .channel_code = 0
    }
};


Channel::Limits default_wl_limits{0, 100, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_vref_limits{2900, 3400, false, Channel::Units::VOLTAGE,  1.0};
Channel::Limits default_ts_limits{0, 100, false, Channel::Units::TEMPERATURE,  1.0};

DataSampler::Channels DataSampler::mChannels = {
       //Lower channels
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
		//2 calibration channels for lower mux part
		Channel(CHANNEL_WL_LREF_MIN, gChannelConfig[CHANNEL_WL_LREF_MIN], Channel::Type::CAL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_LREF_MAX, gChannelConfig[CHANNEL_WL_LREF_MAX], Channel::Type::CAL,   default_wl_limits,   DataSampler::OnChannelLimit),
		 //Upper channels
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
		//2 calibration channels for upper mux part
		Channel(CHANNEL_WL_HREF_MIN, gChannelConfig[CHANNEL_WL_HREF_MIN], Channel::Type::CAL,   default_wl_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_WL_HREF_MAX, gChannelConfig[CHANNEL_WL_HREF_MAX], Channel::Type::CAL,   default_wl_limits,   DataSampler::OnChannelLimit),
        //VREF and temperature channels
		Channel(CHANNEL_TS,  gChannelConfig[CHANNEL_TS],  Channel::Type::TS,   default_ts_limits,   DataSampler::OnChannelLimit),
		Channel(CHANNEL_VREF, gChannelConfig[CHANNEL_VREF], Channel::Type::VREF, default_vref_limits, DataSampler::OnChannelLimit),
		Channel(CHANNEL_VBAT, gChannelConfig[CHANNEL_VBAT], Channel::Type::VBAT, default_vref_limits, DataSampler::OnChannelLimit)
};

DataSampler::Multiplexer DataSampler::mMultiplexer({SCH0, SCH1, SCH2, SCH3});

AdcMode DataSampler::mAdcMode = AdcMode::SE;

struct CalibrationChannels {
	ChannelIdx lo;
	ChannelIdx hi;
} gCalibrationChannels[MUX_BLOCKS] = {{CHANNEL_WL_LREF_MIN,	CHANNEL_WL_LREF_MAX}, {CHANNEL_WL_HREF_MIN,	CHANNEL_WL_HREF_MAX}};

extern Adc_t  AdcVref;
extern Adc_t  AdcVbat;
extern Adc_t  AdcTempSens;
extern Gpio_t SensorsEn[2];
extern Gpio_t SensorsPolarity;
extern Adc_t  AdcInP;
//extern Adc_t  AdcInN;

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

template<unsigned int Bits>
Multiplexer<Bits>::Multiplexer(const std::array<PinNames, Bits>& names):
	mPins{},
	mPinNames{names},
	mConfig()
{
	for(uint8_t bit = 0; bit < Bits; bit++){
		GpioInit( &mPins[bit], mPinNames[bit], PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_PULL_UP, 0 );
	}
}
template<unsigned int Bits>
Multiplexer<Bits>::~Multiplexer(void){
	for(uint8_t bit = 0; bit < Bits; bit++){
		GpioInit( &mPins[bit], mPinNames[bit], PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );
	}
}

template<unsigned int Bits>
void Multiplexer<Bits>::Select(const ChannelConfig &config){
	constexpr uint8_t channels = (1 << Bits);
	uint8_t selected_channel = config.channel_code;
	GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), EN_DISABLED);
	GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), EN_DISABLED);
	if(config.channel_code  < channels){
		for(uint8_t bit = 0; bit < Bits; bit++){
			uint8_t val = (config.channel_code   >> bit)& 0x01;
			GpioWrite(&mPins[bit], val);
		}
		switch(config.channel_en_pin){
			case EN0: {
				GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), EN_DISABLED);
				GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), EN_ACTIVE);
			}
			break;
			case EN1: {
				GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), EN_DISABLED);
				GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), EN_ACTIVE);
				selected_channel+=10;
			}
			break;
			default:{
				assert(0);
			}
		}
		mConfig = &config;
		DBG("ADC Channel %i selected\n", selected_channel);
		osDelay(std::max(MUX_ENABLE_TIMEOUT, MUX_SELECT_TIMEOUT));
	}else {
		assert(0);
	}
}

template<unsigned int Bits>
uint8_t Multiplexer<Bits>::CurrentChannel() const{
	uint8_t channel=0;
	for(uint8_t bit = 0; bit < Bits; bit++){
		channel |= GpioRead(const_cast<Gpio_t*>(&mPins[bit])) << bit;

	}
	return channel;
}
template<unsigned int Bits>
void Multiplexer<Bits>::Sleep(){
	GpioWrite(const_cast<Gpio_t*>(&SensorsEn[0]), EN_DISABLED);
	GpioWrite(const_cast<Gpio_t*>(&SensorsEn[1]), EN_DISABLED);
}

/* End of Multiplexor class */

Channel::Channel(const ChannelIdx id, const ChannelConfig &config, const Type type, const Limits limits, const OnLimit onLimit):
		idx(id),
		type(type),
		limits(limits),
		onLimit(onLimit),
		config(config),
		calibration(type==Type::WL),
		connection()
{

}

void Channel::Measure(Channel::ValueType &val, size_t averaging){
	val = Measure(averaging);
	if((val > limits.hi || val) < limits.lo && onLimit)
		onLimit(this);
}

void Channel::ToggleCurrentDirection(uint16_t time_delay) {

		for (int i = 0; i < 5; i++) {
			GpioToggle(const_cast<Gpio_t*>(&SensorsPolarity));
			osDelay(time_delay);
		}
		GpioWrite(const_cast<Gpio_t*>(&SensorsPolarity), SENS_POL_DIRECT);
		osDelay(time_delay);
}

Channel::ValueType Channel::Measure(size_t averaging) {
    Channel::ValueType result,voltage,resistense,temperature = 0;
    uint16_t total = 0;
#ifdef ADC_OVS_SOFT
    uint16_t resolution = 12; // Разрядность АЦП
    uint16_t oversampling_bits = 2;
    uint16_t oversampling_factor = std::pow(4, oversampling_bits) - 1; // Фактор оверсэмлинга (4^bit) - используем белый шум потому 4
    uint16_t new_resolution = resolution + oversampling_bits; // Новое разрешение (14 бит)
    uint16_t oversampling_devider = std::pow(2, oversampling_bits);
    //Fovs = fADCmax/(2.4*oversampling_bits) //зменение частоты
    uint16_t max_value = (1 << new_resolution) - 1; // Максимальное значение для нового разрешения
#else
    uint16_t max_value = (1 << ADC_OVS_BITS) - 1;
#endif

    if(type ==Type::WL||
       type ==Type::CAL) {
		DataSampler::mMultiplexer.Select(config);
		if(type ==Type::WL) {
			ToggleCurrentDirection(MUX_POL_SWITCH_TIMEOUT);
		}else{
			osDelay(2*MUX_POL_SWITCH_TIMEOUT);
		}
    }

#if(ADC_OVS_SOFT)
    for (int i = 0; i < oversampling_factor; i++) {
#endif
    	if(type == Type::WL || type == Type::CAL) {
			// Считывание значения АЦП
    		if (DataSampler::mAdcMode==AdcMode::SE)
    			total+= AdcReadChannel(const_cast<Adc_t*>(&AdcInP), DataSampler::mAdcMode, averaging);
    		else {
#if 0
    			total+= (AdcReadChannel(const_cast<Adc_t*>(&AdcInP), AdcMode::SE, averaging) - AdcReadChannel(const_cast<Adc_t*>(&AdcInN), AdcMode::SE, averaging));
#endif
    		}
    	}else{
    		total+= AdcReadChannel(type == Type::TS? &AdcTempSens :
    				               type == Type::VREF? &AdcVref : &AdcVbat, AdcMode::SE, averaging);
    	}
#if(ADC_OVS_SOFT)
    }
    uint16_t average = total/oversampling_factor; //calc just average as result already shifted by hardware
#else
    //Сдвиг вправо для получения нового разрешения (14 бит)
    uint16_t average = std::clamp((int)total, 0, (int)max_value);
#endif
    switch(type){
        case Channel::Type::VBAT:
		case Channel::Type::CAL:
		case Channel::Type::WL:{
			voltage =  ADC_CALC_DATA_TO_VOLTAGE(DataSampler::vdda_voltage, average, ADC_OVS_BITS);
			resistense = (voltage / CHANNEL_NOMINAL_CURRENT) *  1000;
			result = (type == Channel::Type::VBAT)? voltage * 3 : resistense;
			connection.state = (type == Channel::Type::CAL)||(type == Channel::Type::VBAT) ? Connection::CONNECTED : connection.state;
		}
		break;
		case Channel::Type::VREF:{
				//not needed - just placeholder
			voltage =  ADC_CALC_VREFANALOG_VOLTAGE(average, ADC_OVS_BITS);
			result = voltage;
			connection.state = Connection::CONNECTED;
		}
		break;
		case Channel::Type::TS:{
			temperature = calc_temperature(DataSampler::vdda_voltage,
													 average,
													 14);
			connection.state = Connection::CONNECTED;
			result = temperature;
		}
		break;
		default:{
			assert(0);
		}
    }
    value = calibration.requred? (result - calibration.shift) * calibration.factor : result;
    react(MeasureEvent(*this, value));
    return value;
}


bool Channel::react(const CalibrationStatusEvent& e){
	if((idx < e.channel.idx) && (idx >= e.channel.idx - CHANNELS_PER_MUX_BLOCK)){
		calibration.react(e);
		return true;
	}else
		return false;
}

void Channel::react(const MeasureEvent& e){
	connection.react(e);
}

void Channel::Connection::react(MeasureEvent const &e){
	if(e.value < CHANNEL_SHORTED_LIMIT)
		state = SHORTED;
	else if (e.value > CHANNEL_DISCONNECTED_LIMIT)
		state = DISCONNECTED;
	else
		state = CONNECTED;
}

void Channel::Calibration::react(CalibrationStatusEvent const &e){
	status = e.status;
	time = std::chrono::system_clock::now();
	if(e.status == PASSED){
		factor = e.factor;
		shift = e.shift;
		error = e.error;
	}
}

std::chrono::seconds Channel::Calibration::elapsedTime(){
	return  std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - time);
}
/* End of Channel class*/

class SelfTest
: public DataSampler {
public:
	void entry() override {
		uint16_t total = 0;
		mTested = std::chrono::system_clock::now();
		DBG("ADC Sampler self test started\n");
//Test selector pcb traces and multiplexor inputs (not open drain init with pullups)
		for(auto &ch:mChannels) {
			if(ch.type == Channel::Type::WL) {
				mMultiplexer.Select(ch.config);
				if(ch.config.channel_code != mMultiplexer.CurrentChannel()){
					dispatch(SelfTestStatusEvent(Status::FAILED));
				}
			}
		}

#ifdef ADC_OVS_SOFT
		uint16_t resolution = 12; // Разрядность АЦП
	    uint16_t oversampling_bits = 2;
		uint16_t oversampling_factor = std::pow(4, oversampling_bits); // Фактор оверсэмлинга (4^2) - используем белый шум потому 4
		uint16_t new_resolution = resolution + oversampling_bits; // Новое разрешение (14 бит)
		uint16_t oversampling_devider = std::pow(2, oversampling_bits);
		uint16_t max_value = (1 << new_resolution) - 1; // Максимальное значение для нового разрешения



		//Initial vdda calibration

		for(int i=0; i< oversampling_factor - 1; i++) {
#endif
			total += AdcReadChannel(&AdcVref, AdcMode::SE, 5);

#ifdef ADC_OVS_SOFT
		}
		total/=oversampling_devider;
#endif
		vdda_voltage = ADC_CALC_VREFANALOG_VOLTAGE(total, ADC_OVS_BITS);//mV
		if(vdda_voltage > VDDA_MIN && vdda_voltage < VDDA_MAX) {
			DBG("VDDA: %i\r\n", vdda_voltage);
			dispatch(SelfTestStatusEvent(Status::PASSED));
		}else {
			vdda_voltage = 0;
			DBG("vdda %i - exceed limits\r\n", vdda_voltage);
			dispatch(SelfTestStatusEvent(Status::FAILED));
	   }
	  }
	void exit(void)  { };
};

class SelfTestFailed
: public DataSampler{
public:
	void entry() override {
		DBG("ADC Sampler self test failed\n");
	}
	void exit(void)  { };
	void react(const SelfTestStatusEvent &e){

	}
};


class Calibrating
: public DataSampler{
	void entry() override {
		float shift = 0;
		float factor = 1.0;
		Channel::ValueType resistence[MUX_BLOCKS];
		mCalibrated = std::chrono::system_clock::now();

		DBG("ADC Calibration Started\n");

		for(auto &cb:gCalibrationChannels) {
			int average = 0;
			memset(resistence, 0, sizeof(resistence));
			while(average++ < CALIBRATION_AVERAGE){
				resistence[0] += mChannels[cb.lo].Measure(CALIBRATION_AVERAGE);
				resistence[1] += mChannels[cb.hi].Measure(CALIBRATION_AVERAGE);
			};
			resistence[0]/=CALIBRATION_AVERAGE;
			resistence[1]/=CALIBRATION_AVERAGE;
			if((resistence[1] && resistence[0]) &&
			   (resistence[1] > resistence[0])){

				shift = REFERENCE_RES_LO - resistence[0];
				float factor = (REFERENCE_RES_HI - REFERENCE_RES_LO)/((resistence[1] - shift) - (resistence[0] - shift));
				float real_lo = factor * (resistence[0] + shift);
				float real_hi = factor * (resistence[1] + shift);
				float error_lo = std::abs((REFERENCE_RES_LO/real_lo) * 100 - 100);
				float error_hi = std::abs((REFERENCE_RES_HI/real_hi) * 100 - 100);
				float error = std::max(error_hi , error_lo)/2;

				if((std::abs(shift) < MAX_SHIFT_ERROR) && (error <=  MAX_ERROR)) {
					dispatch(CalibrationStatusEvent{PASSED, mChannels[cb.lo], shift, factor, error});
				}else{
					dispatch(CalibrationStatusEvent{FAILED, mChannels[cb.lo], shift, factor, error});
				}
			}else{
				dispatch(CalibrationStatusEvent{FAILED, mChannels[cb.lo], shift, factor, MAX_ERROR});
			}
		}
		dispatch(CalibrationEndedEvent());
	}
	void exit(void)  { };
	void react(const SelfTestStatusEvent &e){

	}
};

class CalibrationFailed
: public DataSampler{
	void entry() override {
		DBG("ADC Calibration Failed\n");
	}
	void exit(void)  { };

	void react(const SelfTestStatusEvent &e){

	}
	void react(const CalibrationStatusEvent &e){

	}
};

void SamplerTask(void * argument);
const osThreadAttr_t thread_attr = {
  .name = "DataSampler",
  .stack_size = 128 * 4,                            // Create the thread stack with a size of 1024 bytes
  .priority = (osPriority_t) osPriorityNormal
};

const osMemoryPoolAttr_t samples_attr = {
		.name = "samples"
};

class Sampling
: public DataSampler{
public:
	Sampling(){
		mTaskHandle = osThreadNew(SamplerTask, this, &thread_attr);
	}
	virtual ~Sampling(){
		osMemoryPoolDelete(mSamplesMp);
		osMessageQueueDelete(mSamplesMq);
		osThreadTerminate(mTaskHandle);
	}
	void entry() override {
		DBG("ADC Sampling Started\n");
	}
	void exit(void)  {	}
};

uint16_t DataSampler::vdda_voltage = 0;

DataSampler::DataSampler():
		tinyfsm::Fsm<DataSampler>(),
		mCalibrated(),
		mTested(),
		mSamplesMp(),
		mSamplesMq(),
		mMav(),
		mTs(),
		mSamplePeriod(DEFAULT_SAMPLE_PERIOD),
		mSamplePeriodReal(0),
		mSelfTest(),
		mTaskHandle()
{

}

DataSampler::~DataSampler(){
	DeInit();
}

void DataSampler::DeInit(){

	AdcDeInit( &AdcInP );
//	AdcDeInit( &AdcInN );
}

SamplerMode DataSampler::Mode() {
	return (BoardGetPowerSource() == EXT_POWER)? CONTINUOUS: ONESHOT;
}


void DataSampler::DoSamplerTask()
{
	DBG("ADC Sampler started\n");
	start();
//	BoardInitWatchdog();
	mSamplesMp = (MemPool_t*)osMemoryPoolNew(2, sizeof(struct Samples), &samples_attr);
	mSamplesMq = osMessageQueueNew(1, sizeof(Samples*), nullptr);
	while(1){
		if(is_in_state<Sampling>()){
			Samples* samples = static_cast<Samples*>(new(osMemoryPoolAlloc(mSamplesMp, osWaitForever)) Samples);
			//memset(sensorsData, 0, sizeof(Samples));

			if(samples) {
				for (auto& channel: mChannels) {
					if(!channel.calibration.requred ||
					   (channel.calibration.requred &&
					    channel.calibration.status == PASSED &&
					    channel.calibration.elapsedTime().count() < CALIBRATION_VALID_TIME)){
						channel.Measure((*samples)[channel.idx], SAMPLING_AVERAGE);
					} else {
						osMemoryPoolFree(mSamplesMp, samples);
						samples = NULL;
						Calibrate();
						break;
					}
				}
				mSamplePeriodReal = samples->sampled();
				//Disable multiplexers to reduce power consumptions
				mMultiplexer.Sleep();
				if(samples) {
					*samples = mMav.Filter(samples);
					vdda_voltage = (vdda_voltage + samples->data.ch.Vref)/2.0;//mV;

					for(auto &cb:gCalibrationChannels) {
						float shift = REFERENCE_RES_LO - samples->data.raw[cb.lo];
						float factor = (REFERENCE_RES_HI - REFERENCE_RES_LO)/((samples->data.raw[cb.hi] - shift) - (samples->data.raw[cb.lo] - shift));
						float adc_err = (((vdda_voltage/1000) / std::pow(2,14)) / 2) * 100;
						float real_lo = factor * (samples->data.raw[cb.lo] + shift);
						float real_hi = factor * (samples->data.raw[cb.hi] + shift);
						float error_lo = std::abs((REFERENCE_RES_LO/real_lo) * 100 - 100);
						float error_hi = std::abs((REFERENCE_RES_HI/real_hi) * 100 - 100);
						float error = std::max(error_hi , error_lo)/2;
                        if(error < MAX_ERROR)
                        {
                        	dispatch(CalibrationStatusEvent{PASSED, mChannels[cb.lo], shift, factor, error});
                        }

					}
					mTs = samples->timestamp;
					//calibration update
					for (auto& channel: mChannels) {
						if(channel.calibration.requred &&
						   channel.calibration.elapsedTime().count() > CALIBRATION_VALID_TIME){

						}
					}
#if DEBUG_SAMPLING
					osMemoryPoolFree(mSamplesMp, samples);
#else
					osMessageQueuePut(mSamplesMq, &samples, 0, osWaitForever);
#endif
					}
			}

		}else if(is_in_state<CalibrationFailed>()){
			if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - mCalibrated).count() > CALIBRATION_REPEAT){
				Calibrate();
			}
		}else if(is_in_state<SelfTestFailed>()){
			if(std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - mTested).count() > SELF_TEST_REPEAT){
				SelfTest();
			}
		}else{

		}
		auto period =( (mSamplePeriodReal - mSamplePeriod).count() > 0)? 0 : (mSamplePeriodReal - mSamplePeriod).count();
		osDelay(period);
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

void DataSampler::Calibrate(){
	transit<Calibrating>();
}

void DataSampler::StartSelfTest(){
	transit<SelfTest>();
}


void DataSampler::react(tinyfsm::Event &e) {
	for(auto &ch:mChannels){
		ch.react(e);
	}
};

void DataSampler::react(const SelfTestStatusEvent &e){

	mSelfTest = e.result;
	if(e.result.status == PASSED){
		transit<Calibrating>();
	}else {
		transit<SelfTestFailed>();
	}
}

void DataSampler::react(const CalibrationStatusEvent &e){
	for(auto &ch:mChannels){
		if(e.channel.idx - 10 > ch.idx)continue;
		else ch.react(e);
	}
}

void DataSampler::react(const CalibrationEndedEvent &e){
	int passed_count = 0;
	for(auto &ch:mChannels){
		passed_count += ch.calibration.status==PASSED? 1:0;
	}
	if(passed_count >= WL_CHANNEL_HALF_COUNT){
		transit<Sampling>();
	}else{
		transit<CalibrationFailed>();
	}
}

void DataSampler::react(const SampleEvent &e){

}

void DataSampler::react(const SampleDoneEvent &e){
}

FSM_INITIAL_STATE(DataSampler, SelfTest)
