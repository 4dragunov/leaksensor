#include <memory>
#include <cassert>

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"
#include "NvmDataMgmt.h"
#include "sensors.h"

extern NvProperty<std::underlying_type<OneWire::DS18B20::Resolution>::type> ds18b20_resolution;
extern NvProperty<uint8_t> gActiveRegion;
extern NvProperty<uint32_t> gLoraAppTxDutyCycle;
extern NvProperty<uint16_t> gLoraAppTxDutyCycleRnd;
extern NvProperty<uint8_t> gLoraDefaultDatarate;
extern NvProperty<uint8_t> gLoraAppPort;
extern NvProperty<uint8_t> gLoraAdrState;

extern uint8_t ds18b20Sensors;
extern int16_t ds18b20SensorTemp[8];
extern uint16_t gLeakSensorData[20];
extern uint8_t  gLeakSensorCount;

extern osMailQId  gSummarySensorsMq;

extern Uart_t Usart2;

std::unique_ptr<ModBus::Slave> gSlave;

enum class Index:std::underlying_type_t<ModBus::Index>{
			DS18B20_RESOLUTION = static_cast<std::underlying_type_t<ModBus::Index>>(ModBus::Index::END),
			LORA_REGION,
			LORA_TX_DUTYCYCLE,
			LORA_TX_DUTYCYCLE_RND,
			LORA_DEFAULT_DATARATE,
			LORA_ADR_STATE,
			LORA_APP_PORT,
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
			COUNT,
			END = COUNT
		};
typedef enum_iterator<ModBus::Register::Index, static_cast<ModBus::Register::Index>(Index::WL_1), static_cast<ModBus::Register::Index>(Index::WL_20)> wl_iterator;
typedef enum_iterator<ModBus::Register::Index, static_cast<ModBus::Register::Index>(Index::TEMP_1), static_cast<ModBus::Register::Index>(Index::TEMP_8)> temp_iterator;

ModBus::Registers modbusRegisters = {
		{static_cast<ModBus::Register::Index>(Index::DS18B20_RESOLUTION), ModBus::Register(static_cast<ModBus::Register::Index>(Index::DS18B20_RESOLUTION), {std::ref(ds18b20_resolution)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_REGION),           ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_REGION), {std::ref(gActiveRegion)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_TX_DUTYCYCLE),     ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_TX_DUTYCYCLE), {std::ref(gLoraAppTxDutyCycle)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_TX_DUTYCYCLE_RND), ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_TX_DUTYCYCLE_RND), {std::ref(gLoraAppTxDutyCycleRnd)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_DEFAULT_DATARATE), ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_DEFAULT_DATARATE), {std::ref(gLoraDefaultDatarate)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_ADR_STATE),        ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_ADR_STATE), {std::ref(gLoraAdrState)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{static_cast<ModBus::Register::Index>(Index::LORA_APP_PORT),         ModBus::Register(static_cast<ModBus::Register::Index>(Index::LORA_APP_PORT), {std::ref(gLoraAppPort)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},

		{static_cast<ModBus::Register::Index>(Index::TEMP_SENSORS),          ModBus::Register(static_cast<ModBus::Register::Index>(Index::TEMP_SENSORS), {ModBus::Register::RefValue<uint8_t>{ds18b20Sensors, 0, 8}}, ModBus::Register::Access::RO,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		}
};


void StartTaskModBus(void const * argument)
{
  /* USER CODE BEGIN StartTaskModBus */

	ModBus::Register::Index Index;

	for(size_t sensor = 0; sensor < ds18b20Sensors ; sensor++){
		Index = static_cast<ModBus::Register::Index>(to_underlying(Index::TEMP_1) + sensor);
		modbusRegisters.emplace(Index,
				ModBus::Register(Index, {ModBus::Register::RefValue<int16_t>(ds18b20SensorTemp[sensor], -550, 1250)}, ModBus::Register::Access::RO));
	}
	Index = static_cast<ModBus::Register::Index>(Index::WL_SENSORS);
	modbusRegisters.emplace(Index, ModBus::Register(Index, {ModBus::Register::RefValue<uint8_t>(gLeakSensorCount, 1,  20)}, ModBus::Register::Access::RO));

	for(size_t sensor = 0; sensor < gLeakSensorCount; sensor++) {
		Index = static_cast<ModBus::Register::Index>(to_underlying(Index::WL_1) + sensor);
		modbusRegisters.emplace(Index,
				ModBus::Register(Index, {ModBus::Register::RefValue<uint16_t>(gLeakSensorData[sensor], 0,100)}, ModBus::Register::Access::RO));
	}

	Gpio_t dePin;
	GpioInit(&dePin, PD_4, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_PULL_UP, 0 );
	gSlave = std::make_unique<ModBus::Slave>(&Usart2, &dePin, 1, modbusRegisters);
    assert(gSlave);
    Eeprom &eeprom = Eeprom::Instance();
	gSlave->Start();
	 /* Infinite loop */
	for(;;)
	{
		osEvent ev = osMailGet(gSummarySensorsMq, osWaitForever);
		if(ev.status == osEventMail) {
			DBG("Free eeprom %i\n", eeprom.free);
			DBG("MB MAIL\n");
			SummarySensorsData *s = static_cast<SummarySensorsData *>(ev.value.p);
			size_t j = 0;
			for(auto i :wl_iterator()) {
				(*gSlave)[(int)i] = (s->leakSamples.data.ch.wl[j++]);
			}
			j = 0;
			for(auto i :temp_iterator()) {
				(*gSlave)[(int)i] = s->thermal.data[j++];
			}
			osMailFree(gSummarySensorsMq, ev.value.p);
		}
	}
  /* USER CODE END StartTaskModBus */
}

osThreadId gModbusTaskHandle;
osThreadDef(modbusTask, StartTaskModBus, osPriorityNormal, 0, 512);

void InitModBus(void){
	gModbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);
	assert(gModbusTaskHandle);
}
