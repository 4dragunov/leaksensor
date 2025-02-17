#include <memory>
#include <cassert>

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"
#include "NvmDataMgmt.h"
#include "modbusnode.h"
#include "sensors.h"
#include "cmsis_os.h"

extern NvProperty<std::underlying_type<OneWire::DS18B20::Resolution>::type> ds18b20_resolution;
extern NvProperty<uint8_t> gActiveRegion;
extern NvProperty<uint32_t> gLoraAppTxDutyCycle;
extern NvProperty<uint16_t> gLoraAppTxDutyCycleRnd;
extern NvProperty<uint8_t> gLoraDefaultDatarate;
extern NvProperty<uint8_t> gLoraAppPort;
extern NvProperty<uint8_t> gLoraAdrState;

#define DS18B20MAX_SENSORS 8
extern uint8_t ds18b20Sensors;
extern int16_t ds18b20SensorTemp[8];


extern osMailQId  gSummarySensorsMq;

extern Uart_t Usart2;

enum class Index:std::underlying_type_t<ModBus::Index>{
			DS18B20_RESOLUTION = static_cast<std::underlying_type_t<ModBus::Index>>(ModBus::Index::END),// reg 3
			LORA_REGION, 			//rw - mnd	// reg 4
			LORA_TX_DUTYCYCLE,		//rw - mnd	// reg 5
			LORA_TX_DUTYCYCLE_RND,	//rw - mnd	// reg 6
			LORA_DEFAULT_DATARATE,	//rw - mnd	// reg 7
			LORA_ADR_STATE,			//rw - mnd	// reg 8
			LORA_APP_PORT,			//rw - mnd	// reg 9
			TEMP_SENSORS,			//ro - mnd	// reg 10
			TEMP_1,		 			//ro - mnd	// reg 11
			TEMP_2,		 			//ro - opt	// reg 12
			TEMP_3,					//ro - opt	// reg 13
			TEMP_4,					//ro - opt	// reg 14
			TEMP_5,					//ro - opt	// reg 15
			TEMP_6,					//ro - opt	// reg 16
			TEMP_7,					//ro - opt	// reg 17
			TEMP_8,					//ro - opt	// reg 18
			WL_SENSORS,				//ro - mnd	// reg 19
			WL_1,					//ro - mnd	// reg 20
			WL_2,					//ro - mnd	// reg 21
			WL_3,					//ro - mnd	// reg 22
			WL_4,					//ro - mnd	// reg 23
			WL_5,					//ro - mnd	// reg 24
			WL_6,					//ro - mnd	// reg 25
			WL_7,					//ro - mnd	// reg 26
			WL_8,					//ro - mnd	// reg 27
			WL_9,					//ro - mnd	// reg 28
			WL_10,					//ro - mnd	// reg 29
			WL_12,					//ro - mnd	// reg 30
			WL_13,					//ro - mnd	// reg 31
			WL_14,					//ro - mnd	// reg 32
			WL_15,					//ro - mnd	// reg 33
			WL_16,					//ro - mnd	// reg 34
			WL_17,					//ro - mnd	// reg 35
			WL_18,					//ro - mnd	// reg 36
			WL_19,					//ro - mnd	// reg 37
			WL_20,					//ro - mnd	// reg 38
			COUNT,
			END = COUNT
		};
uint8_t  gLeakSensorCount = 20;
uint16_t gLeakSensorData[20];

typedef enum_iterator<ModBus::Register::Index, static_cast<ModBus::Register::Index>(Index::WL_1), static_cast<ModBus::Register::Index>(Index::WL_20)> wl_iterator;
typedef enum_iterator<ModBus::Register::Index, static_cast<ModBus::Register::Index>(Index::TEMP_1), static_cast<ModBus::Register::Index>(Index::TEMP_8)> temp_iterator;

ModBus::Registers modbusRegisters = {
		{MODBUS_REGISTER(Index::DS18B20_RESOLUTION, {std::ref(ds18b20_resolution)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_REGION, {std::ref(gActiveRegion)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_TX_DUTYCYCLE, {std::ref(gLoraAppTxDutyCycle)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_TX_DUTYCYCLE_RND, {std::ref(gLoraAppTxDutyCycleRnd)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_DEFAULT_DATARATE, {std::ref(gLoraDefaultDatarate)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_ADR_STATE, {std::ref(gLoraAdrState)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::LORA_APP_PORT,{std::ref(gLoraAppPort)}, ModBus::Register::Access::RW,
				[](const ModBus::Register::ValuesType &nvp)->uint16_t
				{
					return std::get<ModBus::Register::nv8_ref>(nvp[0]).get();
				},
				[](ModBus::Register::ValuesType &nvp, const uint16_t value)->uint16_t {
					std::get<ModBus::Register::nv8_ref>(nvp[0]).get() = value;
					return value;
				})
		},
		{MODBUS_REGISTER(Index::TEMP_SENSORS, {ModBus::Register::RefValue<uint8_t>{ds18b20Sensors COMMA 0 COMMA 8}}, ModBus::Register::Access::RO,
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

void ModBusNode::DoTaskModBus()
{
  /* USER CODE BEGIN StartTaskModBus */

	ModBus::Register::Index Index;
	DBG("ModBus task started\n");
	//REg 11 to reg 18
	for(size_t sensor = 0; sensor < DS18B20MAX_SENSORS ; sensor++){
		Index = static_cast<ModBus::Register::Index>(to_underlying(Index::TEMP_1) + sensor);
		DBG("ModBus reg temp:%i created\n", Index);
		modbusRegisters.emplace(Index,
				ModBus::Register(Index, "" ,{ModBus::Register::RefValue<int16_t>(ds18b20SensorTemp[sensor], -550, 1250)}, ModBus::Register::Access::RO));
	}
	Index = static_cast<ModBus::Register::Index>(Index::WL_SENSORS);
	DBG("ModBus reg sensor count:%i created\n", Index);
	modbusRegisters.emplace(Index, ModBus::Register(Index, "", {ModBus::Register::RefValue<uint8_t>(gLeakSensorCount, 1,  20)}, ModBus::Register::Access::RO));

	for(size_t sensor = 0; sensor < gLeakSensorCount; sensor++) {
		Index = static_cast<ModBus::Register::Index>(to_underlying(Index::WL_1) + sensor);
		DBG("ModBus reg wl:%i created\n", Index);
		modbusRegisters.emplace(Index,
				ModBus::Register(Index, "", {ModBus::Register::RefValue<uint16_t>(gLeakSensorData[sensor], 0,100)}, ModBus::Register::Access::RO));
	}

	mSlave->Start();
	 /* Infinite loop */
	for(;;)
	{
		if(osSemaphoreAcquire(mDataChangedSem, osWaitForever) == osOK) {
			DBG("MB MAIL\n");
			std::shared_ptr<SummarySensorsData> sensorData = static_pointer_cast<SummarySensorsData>(mSensorData);
			size_t j = 0;
			for(auto i :wl_iterator()) {
				uint16_t data = sensorData->leakSamples.data.ch.wl[j++];
				DBG("Set mb wl reg:%i to %i\n", i, data);
				(*mSlave)[(int)i].SetValue(data);
			}
			DBG("WL-set!\n");
			j = 0;
			for(auto i :temp_iterator()) {
				uint16_t value = sensorData->thermal.data[j++];
				DBG("Set mb temp reg:%i to %i\n", i, value);
				(*mSlave)[(int)i].SetValue(value);
			}
			DBG("TMP-set!\n");
			sensorData = nullptr;
			mSensorData = nullptr;
		}
	}
  /* USER CODE END StartTaskModBus */
}

void StartTaskModBus(void * argument){
	static_cast<ModBusNode*>(argument)->DoTaskModBus();
}

const osThreadAttr_t thread_attr = {
  .name = "ModBusNode",
  .stack_size = 2048                            // Create the thread stack with a size of 1024 bytes
};
ModBusNode::ModBusNode(MessageBus& mbus):
	BusNode(&mbus),
	mModbusTaskHandle(osThreadNew(StartTaskModBus, this, &thread_attr)),
	mSensorData(nullptr),
    mDataChangedSem(osSemaphoreNew(1, 0, nullptr)),
	mDePin(),
	mSlave(std::make_unique<ModBus::Slave>(&Usart2, &mDePin, 1, modbusRegisters))
{
	GpioInit(&mDePin, PD_4, PIN_OUTPUT, PIN_OPEN_DRAIN, PIN_PULL_UP, 0 );
	DBG("%s\r\n",__FUNCTION__);
}

ModBusNode::~ModBusNode(){
}

ModBusNode& ModBusNode::Instance(MessageBus &b)
{
	static ModBusNode n(b);
	return n;
}

void ModBusNode::onNotify(MessageBus::Message &message){
	mSensorData = message;
	DBG("ModBusNode %s\r\n",__FUNCTION__);
	osSemaphoreRelease(mDataChangedSem);
}

ModBusNode& InitModBus(MessageBus& mbus){
	return ModBusNode::Instance(mbus);
}
