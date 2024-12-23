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
#include <math.h>
#include <assert.h>
#include <sys/time.h>

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

#include "OneWire.h"
#include "Ds18B20.h"
#include "modbus.h"

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





/*!
 *
 */
typedef enum
{
    LORAMAC_HANDLER_TX_ON_TIMER,
    LORAMAC_HANDLER_TX_ON_EVENT,
}LmHandlerTxEvents_t;

extern ChannelConfig gChannelConfig[ADC_CHANNEL_COUNT];
extern ChannelPins gChannelsPins[ADC_CHANNEL_COUNT];
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

osThreadId modbusTaskHandle;
uint32_t modbusTaskBuffer[ 128 ];
osStaticThreadDef_t modbusTaskControlBlock;

osThreadId oneWireTaskHandle;
uint32_t oneWireTaskBuffer[ 128 ];
osStaticThreadDef_t oneWireTaskControlBlock;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;



osThreadId timerTestTaskHandle;
uint32_t timerTestTaskBuffer[ 128 ];
osStaticThreadDef_t timerTestTaskControlBlock;

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
    .GetTemperature = NULL,
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

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
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

extern Ds18B20_t ds18b20;

volatile uint16_t channel_data[20] ;


void SetChannels(ChannelPins* active_channel, uint8_t val) {
    // Перебираем все каналы и подаем высокий логический уровень на неактивные
    for (int i = 0; i < 20; i++) {
        if (&gChannelsPins[i] != active_channel) {
            GpioWrite(&gChannelsPins[i].ptp, val);
            GpioWrite(&gChannelsPins[i].ntp, val);
        }
    }
}


void ToggleCurrentDirection(Gpio_t *pin1, Gpio_t *pin2, uint16_t time_delay) {
    for (int i = 0; i < 5; i++) {
        GpioWrite(pin1, 1);
        GpioWrite(pin2, 0);
        osDelay(time_delay);
        GpioWrite(pin1, 0);
        GpioWrite(pin2, 1);
        osDelay(time_delay);
    }
    GpioWrite(pin1, 0);
    GpioWrite(pin2, 0);
    osDelay(time_delay);
}

uint16_t ProcessChannel(ChannelPins* channel) {
    const int num_measurements = 12;
    uint16_t adc_values[num_measurements];
    float sum = 0;
    uint16_t max_value = 0;
    uint16_t min_value = 0xFFFF; // �?нициализируем минимальное значение максимальным возможным

    // Устанавливаем высокий логический уровень на неиспользуемые каналы
    SetChannels(channel, 1);


    for (int i = 0; i < num_measurements; i++) {
        ToggleCurrentDirection(&channel->ptp, &channel->ntp, 30);
        GpioWrite(&gChannelsPins[i].ntp, 1); // подача высокого уровня перед АЦП
       // HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); //led on
        osDelay(10);

        // Считывание значения АЦП

        adc_values[i] = AdcReadChannel( &channel->ap);

        sum += adc_values[i];
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
        }
        if (adc_values[i] < min_value) {
            min_value = adc_values[i];
        }

        GpioWrite(&channel->ptp, 0); // подача низкого уровня перед АЦП
        osDelay(10);
    }

    uint16_t average = (uint16_t)round(sum / num_measurements);
    // Сброс логического уровня на неиспользуемых каналах после измерения
    SetChannels(channel, 0);
    return average;
}


void StartTaskModBus(void const * argument)
{
  /* USER CODE BEGIN StartTaskModBus */
	uint16_t modbusDATA[8];
	Gpio_t dePin;
	GpioInit(&dePin, PD_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
	ModBusSlave modbusSlave(&Usart2, &dePin, 10, modbusDATA, ARRAY_SIZE(modbusDATA));
  /* Infinite loop */

	//modbusSlave.Start();
	for(;;)
	{
		osDelay(500);
		GpioWrite( &Led1, 1 );
		GpioWrite( &Led2, 1 );
		TimerStart( &Led2Timer );
	}
  /* USER CODE END StartTaskModBus */
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
    // Initialize transmission periodicity variable
    TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );

    const Version_t appVersion = { .Value = FIRMWARE_VERSION };
    const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
    DisplayAppInfo( "periodic-uplink-lpp",
                    &appVersion,
                    &gitHubVersion );

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
  /* USER CODE BEGIN StartTaskOneWire */
  /* Infinite loop */
  DS18B20_init(&ds18b20, &Usart1, DS18B20_12BITS);
  uint8_t sensors = DS18B20_getSensorsAvailable(&ds18b20);
  if (sensors)
      DS18B20_startMeasure(&ds18b20, DS18B20_MEASUREALL);
  for(;;)
  {
    osDelay(1000);
    if (sensors) {
      if (DS18B20_isTempReady(&ds18b20, 0)) {
        for (uint8_t s=0;s<sensors;s++) {
            int16_t tempRaw = DS18B20_getTempRaw(&ds18b20, s);
            printf("sensor #%d, temp raw = %d\n", s, tempRaw);
        }
        DS18B20_startMeasure(&ds18b20, DS18B20_MEASUREALL);
      }
    } else {
        printf("no sensors\n");
    }
  }

  /* USER CODE END StartTaskOneWire */
}


void StartTaskLeakMeter(void const * argument)
{
	BoardInitPeriph( );

	while( 1 ){
		// Process characters sent over the command line interface
		for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
			channel_data[i] = ProcessChannel(&gChannelsPins[i]);
		}

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
			BoardLowPowerHandler( );
		}
		CRITICAL_SECTION_END( );
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
    	DBG("tv: sec: %d, usec:%d \n", (uint32_t)tv.tv_sec, (uint32_t)tv.tv_usec);
    }

}
#endif
/*!
 * Main application entry point.
 */
int main( void )
{
    BoardInitMcu( );

/*
    osThreadStaticDef(modbusTask, StartTaskModBus, osPriorityNormal, 0, 128, modbusTaskBuffer, &modbusTaskControlBlock);
    modbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);


    osThreadStaticDef(oneWireTask, StartTaskOneWire, osPriorityNormal, 0, 128, oneWireTaskBuffer, &oneWireTaskControlBlock);
    oneWireTaskHandle = osThreadCreate(osThread(oneWireTask), NULL);

    osThreadStaticDef(defaultTask, StartTaskDefault, osPriorityNormal, 0, 128, defaultTaskBuffer, &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

 */
    osThreadStaticDef(timerTestTask, StartTaskTimerTest, osPriorityNormal, 0, 128, timerTestTaskBuffer, &timerTestTaskControlBlock);
    timerTestTaskHandle = osThreadCreate(osThread(timerTestTask), NULL);


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
    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    uint8_t channel = 0;

    AppData.Port = LORAWAN_APP_PORT;

    CayenneLppReset( );
    CayenneLppAddDigitalInput( channel++, AppLedStateOn );

    for(int i = 0; i < 20; i++)
    	CayenneLppAddAnalogInput( channel++, channel_data[i] * 100 / 254 );

    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );

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
        TxPeriodicity = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
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
