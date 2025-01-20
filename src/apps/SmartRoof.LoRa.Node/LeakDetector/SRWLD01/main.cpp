/*!
 * \file      main.c
 *
 * \brief     Performs a periodic uplink
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2018 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 */

/*! \file periodic-uplink/NucleoL152/main.c */

#include <stdio.h>
#include <memory>
#include <math.h>
#include <assert.h>
#include <sys/time.h>
#include <time.h>

#include "../../../../boards/SRWLD01/mav.h"
#include "../firmwareVersion.h"
#include "../../common/githubVersion.h"
#include "utilities.h"
#include "board.h"
#include "gpio.h"
#include "uart.h"
#include "RegionCommon.h"

#include "cli.h"
#include "Commissioning.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "CayenneLpp.h"
#include "LmHandlerMsgDisplay.h"

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"

#define TEST

using namespace ModBus::Slave;

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#ifndef LORAWAN_DEFAULT_CLASS
#define LORAWAN_DEFAULT_CLASS                       CLASS_A
#endif

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON


/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

/*!
 * LoRaWAN application port
 * @remark The allowed port range is from 1 up to 223. Other values are reserved.
 */
#define LORAWAN_APP_PORT                            2


#define VREF        3.3   // Опорное напряжение АЦП
#define ADC_MAX     4095   // Максимальное значение для 12-битного АЦП
#define R1          10000  // Значение известного сопротивления R1 в омах
#define UART_BUFFER_SIZE 256


NvProperty<uint8_t> gActiveRegion(LORAMAC_REGION_AS923, LORAMAC_REGION_RU864, ACTIVE_REGION, NvVar::LORA_REGION);

NvProperty<std::underlying_type<OneWire::DS18B20::Resolution>::type> ds18b20_resolution(to_underlying(OneWire::DS18B20::Resolution::SR9BITS),
		to_underlying(OneWire::DS18B20::Resolution::SR12BITS),
		to_underlying(OneWire::DS18B20::Resolution::SR12BITS), NvVar::DS18B20_RESOLUTION);
//2678400000
NvProperty<uint32_t> gLoraAppTxDutyCycle(1000,  1000u * 60u * 60u * 24u* 31u, APP_TX_DUTYCYCLE, NvVar::LORA_TX_DUTYCYCLE);

NvProperty<uint16_t> gLoraAppTxDutyCycleRnd(100,  1000 * 2, APP_TX_DUTYCYCLE_RND, NvVar::LORA_TX_DUTYCYCLE_RND);
NvProperty<uint8_t> gLoraAdrState(0,  1, LORAWAN_ADR_STATE, NvVar::LORA_ADR_STATE);

NvProperty<uint8_t> gLoraDefaultDatarate(DR_0,  DR_15, LORAWAN_DEFAULT_DATARATE, NvVar::LORA_DEFAULT_DATARATE);
NvProperty<uint8_t> gLoraAppPort(1,  223, LORAWAN_APP_PORT, NvVar::LORA_APP_PORT);

/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;


/*!
 * User application data
 */
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*!
 * User application data structure
 */
static LmHandlerAppData_t AppData =
{

    .Port = 0,
	.BufferSize = 0,
	.Buffer = AppDataBuffer
};

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxTimer;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Timer to handle the state of LED beacon indicator
 */
static TimerEvent_t LedBeaconTimer;

void StartTaskModBus(void const * argument);
osThreadId modbusTaskHandle;
osThreadDef(modbusTask, StartTaskModBus, osPriorityNormal, 0, 256);

void StartTaskOneWire(void const * argument);
osThreadId oneWireTaskHandle;
osThreadDef(oneWireTask, StartTaskOneWire, osPriorityNormal, 0, 256);

void StartTaskDefault(void const * argument);
osThreadId defaultTaskHandle;
osThreadDef(defaultTask, StartTaskDefault, osPriorityNormal, 0, 256);


static uint8_t ds18b20Sensors = 0;
int16_t ds18b20SensorTemp[8] = {};
uint16_t gLeakSensorData[20] = {};
uint8_t  gLeakSensorCount = 20;

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

static void OnMacProcessNotify( void );
static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size );
static void OnNetworkParametersChange( CommissioningParams_t* params );
static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn );
static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn );
static void OnJoinRequest( LmHandlerJoinParams_t* params );
static void OnTxData( LmHandlerTxParams_t* params );
static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params );
static void OnClassChange( DeviceClass_t deviceClass );
static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params );
#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection );
#else
static void OnSysTimeUpdate( void );
#endif
static void PrepareTxFrame( void );
static void StartTxProcess( LmHandlerTxEvents_t txEvent );
static void UplinkProcess( void );

static void OnTxPeriodicityChanged( uint32_t periodicity );
static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed );
static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity );

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context );

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void* context );

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void* context );

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context );

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = BoardGetInternalTemperature,
    .GetRandomSeed = BoardGetRandomSeed,
    .OnMacProcess = OnMacProcessNotify,
    .OnNvmDataChange = OnNvmDataChange,
    .OnNetworkParametersChange = OnNetworkParametersChange,
    .OnMacMcpsRequest = OnMacMcpsRequest,
    .OnMacMlmeRequest = OnMacMlmeRequest,
    .OnJoinRequest = OnJoinRequest,
    .OnTxData = OnTxData,
    .OnRxData = OnRxData,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = OnBeaconStatusChange,
    .OnSysTimeUpdate = OnSysTimeUpdate,
};

static LmHandlerParams_t LmHandlerParams =  {
        .Region = (LoRaMacRegion_t)(uint8_t)gActiveRegion,
        .AdrEnable = gLoraAdrState,
        .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
        .TxDatarate = gLoraDefaultDatarate,
        .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
        .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
        .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
        .DataBuffer = AppDataBuffer,
        .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
    };


static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion = {.Value = FIRMWARE_VERSION},
    .OnTxPeriodicityChanged = OnTxPeriodicityChanged,
    .OnTxFrameCtrlChanged = OnTxFrameCtrlChanged,
    .OnPingSlotPeriodicityChanged = OnPingSlotPeriodicityChanged,
};

/*!
 * Indicates if LoRaMacProcess call is pending.
 * 
 * \warning If variable is equal to 0 then the MCU can be set in low power mode
 */
static volatile uint8_t IsMacProcessPending = 0;

static volatile uint8_t IsTxFramePending = 0;

static volatile uint32_t TxPeriodicity = 0;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx
extern Gpio_t Led2; // Rx

/*!
 * UART object used for command line interface handling
 */
extern Uart_t Usart1;
extern Uart_t Usart2;
extern Adc_t Adc1;
extern Adc_t Adc3;

extern OneWire::Bus gOWI;
extern OneWire::DS18B20 gDs18b20;

typedef struct {
	int16_t data[20];
	uint8_t sensors;
	time_t timestamp;
} LeakSensorsData ;

typedef struct {
	int16_t data[8];
	uint8_t sensors;
	time_t timestamp;
} ThermalSensorsData ;

osMailQDef(ThermalSensors, 1, ThermalSensorsData);                    // Define mail queue
osMailQId  gThermalSensorsMq;

osMailQDef(LeakSensors, 1, LeakSensorsData);                    // Define memory pool
osMailQId  gLeakSensorsMq;


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
	GpioInit(&dePin, PD_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
	auto slave = std::make_unique<ModBusSlave>(&Usart2, &dePin, 10, modbusRegisters);
  /* Infinite loop */

	slave->Start();
	for(;;)
	{

		osEvent ev = osMailGet(DataSampler::Instance(), osWaitForever);



		if(ev.status == osEventMail) {
			Samples *s = static_cast<Samples *>(ev.value.p);
			size_t j = 0;
			for(auto i :wl_iterator()) {
				(*slave)[(int)i] = s[(int)i].data.ch.wl[j++];
			}
			osMailFree(DataSampler::Instance(), ev.value.p);
		}
	}
  /* USER CODE END StartTaskModBus */
}

void UpdateHadlerProps(){

}
/* USER CODE BEGIN Header_StartTaskLeakMeter */
/**
* @brief Function implementing the leakMeterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLeakMeter */
void StartTaskDefault(void const * argument)
{
  /* USER CODE BEGIN StartTaskLeakMeter */
	TimerInit( &Led1Timer, OnLed1TimerEvent );
	TimerSetValue( &Led1Timer, 25 );

	TimerInit( &Led2Timer, OnLed2TimerEvent );
	TimerSetValue( &Led2Timer, 25 );

	TimerInit( &LedBeaconTimer, OnLedBeaconTimerEvent );
	TimerSetValue( &LedBeaconTimer, 5000 );



    const Version_t appVersion = { .Value = FIRMWARE_VERSION };
    const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
    DisplayAppInfo( "periodic-uplink-lpp",
                    &appVersion,
                    &gitHubVersion );




    // Initialize transmission periodicity variable
    TxPeriodicity = gLoraAppTxDutyCycle + randr( -gLoraAppTxDutyCycleRnd, gLoraAppTxDutyCycleRnd );
    LmHandlerParams =   {
        .Region = (LoRaMacRegion_t)(uint8_t)gActiveRegion,
        .AdrEnable = gLoraAdrState,
        .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
        .TxDatarate = gLoraDefaultDatarate,
        .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
        .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
        .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
        .DataBuffer = AppDataBuffer,
        .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
    };




    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        printf( "LoRaMac wasn't properly initialized\n" );
        // Fatal error, endless loop.
        while ( 1 )
        {
        	assert(0);
        }
    }

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError( 20 );

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );

    LmHandlerJoin( );

    StartTxProcess( LORAMAC_HANDLER_TX_ON_TIMER );
  /* Infinite loop */
  for(;;)
  {

    // Process characters sent over the command line interface
          //CliProcess( &Usart2 );
          // Processes the LoRaMac events
          LmHandlerProcess( );

          // Process application uplinks management
          UplinkProcess( );

          CRITICAL_SECTION_BEGIN( );
          if( IsMacProcessPending == 1 )
          {
              // Clear flag and prevent MCU to go into low power modes.
              IsMacProcessPending = 0;
          }
          else
          {
              // The MCU wakes up through events
#ifndef DEBUG
              BoardLowPowerHandler( );
#endif
          }
          CRITICAL_SECTION_END( );
  }
  /* USER CODE END StartTaskLeakMeter */
}



/* USER CODE BEGIN Header_StartTaskOneWire */
/**
* @brief Function implementing the oneWireTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOneWire */
void StartTaskOneWire(void const * argument)
{


  for(;;)
  {
    if(!ds18b20Sensors) {
    	ds18b20Sensors = gDs18b20.init(static_cast<OneWire::DS18B20::Resolution>((uint8_t)ds18b20_resolution));
    	if(!ds18b20Sensors) {DBG("No sensors!\n");osDelay(200);}
    }else {
    	if(gDs18b20.startMeasure(to_underlying(OneWire::DS18B20::Command::MEASUREALL)) == osOK){

    	if(gDs18b20.waitTempReady(0) == osOK) {
    		DBG("Temp ready\n");
    		//gLoraAppPort.store.dump();
    		DBG("Heap %i\n",xPortGetFreeHeapSize());
    		ThermalSensorsData* sensorData = static_cast<ThermalSensorsData*>(osMailAlloc(gThermalSensorsMq, osWaitForever));
    		if(sensorData) {
				for(uint8_t s = 0; s < ds18b20Sensors; s++) {
					OneWire::DS18B20::Error err = gDs18b20.getTempRaw(s, &sensorData->data[s]);
					if(err == OneWire::DS18B20::Error::TEMP_READ){
						DBG("sensor #%d, temp raw = %d\n", s, sensorData->data[s]);
					}
					else{
						DBG("Temp not read\n");
					}
				}
				sensorData->sensors = ds18b20Sensors;
				time(&sensorData->timestamp);
				osMailPut(gThermalSensorsMq, (void*)sensorData);  // Send Message
    		}// Cooperative multitasking

        }else {
        	DBG("temp wait error\n");
        }
	  } else{
		  DBG("failed to start measure\n");
	  }
  }
 }
}


#ifdef TEST
static void keyCallback(void* context ){
    GpioToggle(&Led1);
}
static Gpio_t key;
GpioIrqHandler *keyHandler = keyCallback;

/* USER CODE END Header_StartTaskLeakMeter */
void StartTaskTimerTest(void const * argument)
{
	struct timeval tv;
    GpioInit( &key, PE_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    GpioSetInterrupt( &key, IRQ_FALLING_EDGE, IRQ_HIGH_PRIORITY, keyHandler );

    bool passed = Board_Timer_Test();
    DBG("Timer test passed: %i\n", passed);
    for(;;)
    {
    	osDelay(100);
    	gettimeofday(&tv, NULL);
    	GpioToggle(&Led1);
    	DBG("tv: sec: %ld, usec:%ld \n", (uint32_t)tv.tv_sec, (uint32_t)tv.tv_usec);
    }

}
#endif
/*!
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );
    //Not needed but for compatibility
    osKernelInitialize();   // pre initialize CMSIS-RTOS

    modbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);

    oneWireTaskHandle = osThreadCreate(osThread(oneWireTask), NULL);

    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
    // create mail queues
    gThermalSensorsMq = osMailCreate(osMailQ(ThermalSensors), NULL);

    gLeakSensorsMq = osMailCreate(osMailQ(LeakSensors), NULL);

	DBG("Heap %i\n", xPortGetFreeHeapSize());

    osKernelStart();

    while(1) {
    	assert(0);
    }
}

static void OnMacProcessNotify( void )
{
    IsMacProcessPending = 1;
}

static void OnNvmDataChange( LmHandlerNvmContextStates_t state, uint16_t size )
{
    DisplayNvmDataChange( state, size );
}

static void OnNetworkParametersChange( CommissioningParams_t* params )
{
    DisplayNetworkParametersUpdate( params );
}

static void OnMacMcpsRequest( LoRaMacStatus_t status, McpsReq_t *mcpsReq, TimerTime_t nextTxIn )
{
    DisplayMacMcpsRequestUpdate( status, mcpsReq, nextTxIn );
}

static void OnMacMlmeRequest( LoRaMacStatus_t status, MlmeReq_t *mlmeReq, TimerTime_t nextTxIn )
{
    DisplayMacMlmeRequestUpdate( status, mlmeReq, nextTxIn );
}

static void OnJoinRequest( LmHandlerJoinParams_t* params )
{
    DisplayJoinRequestUpdate( params );
    if( params->Status == LORAMAC_HANDLER_ERROR )
    {
        LmHandlerJoin( );
    }
    else
    {
        LmHandlerRequestClass( LORAWAN_DEFAULT_CLASS );
    }
}

static void OnTxData( LmHandlerTxParams_t* params )
{
    DisplayTxUpdate( params );
}

static void OnRxData( LmHandlerAppData_t* appData, LmHandlerRxParams_t* params )
{
    DisplayRxUpdate( appData, params );
//TODO:rx data process
    switch( appData->Port )
    {
    case 1: // The application LED can be controlled on port 1 or 2
    case LORAWAN_APP_PORT:
        {
            AppLedStateOn = appData->Buffer[0] & 0x01;
        }
        break;
    default:
        break;
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite( &Led2, 1 );
    TimerStart( &Led2Timer );
}

static void OnClassChange( DeviceClass_t deviceClass )
{
    DisplayClassUpdate( deviceClass );

    // Inform the server as soon as possible that the end-device has switched to ClassB
    LmHandlerAppData_t appData =
    {
        .Port = 0,
		.BufferSize = 0,
		.Buffer = NULL,
    };
    LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
}

static void OnBeaconStatusChange( LoRaMacHandlerBeaconParams_t* params )
{
    switch( params->State )
    {
        case LORAMAC_HANDLER_BEACON_RX:
        {
            TimerStart( &LedBeaconTimer );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            TimerStop( &LedBeaconTimer );
            break;
        }
        default:
        {
            break;
        }
    }

    DisplayBeaconUpdate( params );
}

#if( LMH_SYS_TIME_UPDATE_NEW_API == 1 )
static void OnSysTimeUpdate( bool isSynchronized, int32_t timeCorrection )
{

}
#else
static void OnSysTimeUpdate( void )
{

}
#endif

/*!
 * Prepares the payload of the frame and transmits it.
 */
static void PrepareTxFrame( void )
{
	uint8_t channel = 0;
	osEvent evt;

    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset( );

    evt = osMailGet(gLeakSensorsMq, osWaitForever);  // wait for message
    if (evt.status == osEventMail) {
    	LeakSensorsData *leakSensorsData = static_cast<LeakSensorsData*>(evt.value.p);

    	CayenneLppAddDigitalInput(channel++, leakSensorsData->sensors );
    	for(int i = 0; i < 20; i++) {
    		CayenneLppAddRelativeHumidity(channel++, leakSensorsData->data[i] * 100 / 254 );
    	}
    	osMailFree(gLeakSensorsMq, leakSensorsData);
	}

    evt = osMailGet(gThermalSensorsMq, osWaitForever);  // wait for message
    if (evt.status == osEventMail) {
    	ThermalSensorsData *thermalSensorsData = static_cast<ThermalSensorsData *>(evt.value.p);

    	CayenneLppAddDigitalInput(channel++, thermalSensorsData->sensors );

    	for(int i = 0; i < thermalSensorsData->sensors; i++) {
    		CayenneLppAddTemperature( channel++, thermalSensorsData->data[i] * 100 / 254 );
    	}
        osMailFree(gThermalSensorsMq, thermalSensorsData);
    }

    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );
   // CayenneLppAddAnalogOutput( channel++, BoardGetModbusId( ) * 100 / 254 );

    CayenneLppCopy( AppData.Buffer );
    AppData.BufferSize = CayenneLppGetSize( );

    if( LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS )
    {
        // Switch LED 1 ON
        GpioWrite( &Led1, 1 );
        TimerStart( &Led1Timer );
    }
}

static void StartTxProcess( LmHandlerTxEvents_t txEvent )
{
    switch( txEvent )
    {
    default:
        // Intentional fall through
    case LORAMAC_HANDLER_TX_ON_TIMER:
        {
            // Schedule 1st packet transmission
            TimerInit( &TxTimer, OnTxTimerEvent );
            TimerSetValue( &TxTimer, TxPeriodicity );
            OnTxTimerEvent( NULL );
        }
        break;
    case LORAMAC_HANDLER_TX_ON_EVENT:
        {
        }
        break;
    }
}

static void UplinkProcess( void )
{
    uint8_t isPending = 0;
    CRITICAL_SECTION_BEGIN( );
    isPending = IsTxFramePending;
    IsTxFramePending = 0;
    CRITICAL_SECTION_END( );
    if( isPending == 1 )
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
       //TODO:!! TxPeriodicity = gLoraAppTxDutyCycle + randr( -gLoraAppTxDutyCycleRnd, gLoraAppTxDutyCycleRnd );
    }

    // Update timer periodicity
    TimerStop( &TxTimer );
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
    LmHandlerParams.IsTxConfirmed = isTxConfirmed;
}

static void OnPingSlotPeriodicityChanged( uint8_t pingSlotPeriodicity )
{
    LmHandlerParams.PingSlotPeriodicity = pingSlotPeriodicity;
}

/*!
 * Function executed on TxTimer event
 */
static void OnTxTimerEvent( void* context )
{
    TimerStop( &TxTimer );

    IsTxFramePending = 1;

    // Schedule next transmission
    TimerSetValue( &TxTimer, TxPeriodicity );
    TimerStart( &TxTimer );
}

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void* context )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    GpioWrite( &Led1, 0 );
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void* context )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    GpioWrite( &Led2, 0 );
}

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent( void* context )
{
    GpioWrite( &Led2, 1 );
    TimerStart( &Led2Timer );

    TimerStart( &LedBeaconTimer );
}
