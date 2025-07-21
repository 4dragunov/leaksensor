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
#include "NvmDataMgmt.h"
#include "sensors.h"
#include "modbusnode.h"
#include "onewirenode.h"
#include "loranode.h"
#include "cmsis_os.h"

#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#ifndef LORAWAN_DEFAULT_CLASS
#define LORAWAN_DEFAULT_CLASS                       CLASS_B
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

NvProperty<uint8_t>  gActiveRegion(LORAMAC_REGION_EU868, LORAMAC_REGION_RU864, ACTIVE_REGION, NvVar::LORA_REGION);
NvProperty<uint32_t> gLoraAppTxDutyCycle(1000,  1000u * 60u * 60u * 24u* 31u, APP_TX_DUTYCYCLE, NvVar::LORA_TX_DUTYCYCLE);//2678400000
NvProperty<uint16_t> gLoraAppTxDutyCycleRnd(100,  1000 * 2, APP_TX_DUTYCYCLE_RND, NvVar::LORA_TX_DUTYCYCLE_RND);
NvProperty<uint8_t>  gLoraAdrState(0,  1, LORAWAN_ADR_STATE, NvVar::LORA_ADR_STATE);
NvProperty<uint8_t>  gLoraDefaultDatarate(DR_0,  DR_15, LORAWAN_DEFAULT_DATARATE, NvVar::LORA_DEFAULT_DATARATE);
NvProperty<uint8_t>  gLoraAppPort(1,  223, LORAWAN_APP_PORT, NvVar::LORA_APP_PORT);

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
LmHandlerAppData_t AppData =
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
static osTimerId_t TxTimer;

osSemaphoreDef(UplinkSem);
osSemaphoreId gUplinkSem;

void StartTaskDefault(void * argument);
osThreadId defaultTaskHandle;
LoraNode* gLoraNode = nullptr;

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
static void OnLed1TimerEvent(void* context );

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void* context );

/*!
 * Function executed on Led 3 Timeout event
 */
static void OnLed3TimerEvent(void* context );

/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void* context );

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = BoardGetBatteryLevel,
    .GetTemperature = BoardGetTemperature,
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
        .AdrEnable = (bool) gLoraAdrState,
        .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
        .TxDatarate = (int8_t)gLoraDefaultDatarate,
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

static volatile uint32_t TxPeriodicity = 0;

/*!
 * LED GPIO pins objects
 */
extern Gpio_t Led1; // Tx
extern Gpio_t Led2; // Rx
extern Gpio_t Led3;
/*!
 * Timer to handle the state of LED1
 */
static osTimerId_t Led1Timer;
#define LED1_TIMER_TIMEOUT 25
/*!
 * Timer to handle the state of LED2
 */
static osTimerId_t Led2Timer;
#define LED2_TIMER_TIMEOUT 25

/*!
 * Timer to handle the state of LED3
 */
static osTimerId_t Led3Timer;
#define LED3_TIMER_TIMEOUT 25

/*!
 * Timer to handle the state of LED beacon indicator
 */
static osTimerId_t LedBeaconTimer;
#define LED_BEACON_TIMER_TIMEOUT 5000

static struct Leds{
	Gpio_t &pio;
	osTimerId_t &timer;
	uint16_t   timeout;
}gLeds[] = {{Led1, Led1Timer, LED1_TIMER_TIMEOUT}, {Led2, Led2Timer, LED2_TIMER_TIMEOUT}, {Led3, Led3Timer, LED3_TIMER_TIMEOUT}};

#define LED_SWITCH_ON(led) {DBG("LED:%i:%i\n", led, LED_OFF);GpioWrite( &gLeds[led].pio, LED_ON ); osTimerStart( gLeds[led].timer, gLeds[led].timeout );}
#define LED_SWITCH_OFF(led)    {DBG("LED:%i:%i\n", led, LED_OFF);osTimerStop( gLeds[led].timer ); GpioWrite( &gLeds[led].pio, LED_OFF );}

void UpdateHadlerProps(){

}
/* USER CODE BEGIN Header_StartTaskLeakMeter */
/**
* @brief Function implementing the leakMeterTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskLeakMeter */
void StartTaskDefault(void * argument)
{
  /* USER CODE BEGIN StartTaskLeakMeter */

	Led1Timer = osTimerNew( OnLed1TimerEvent, osTimerOnce, NULL, NULL );

	Led2Timer = osTimerNew( OnLed2TimerEvent, osTimerOnce, NULL, NULL );

	Led3Timer = osTimerNew( OnLed3TimerEvent, osTimerOnce, NULL, NULL );

	LedBeaconTimer = osTimerNew( OnLedBeaconTimerEvent, osTimerOnce, NULL, NULL );

    const Version_t appVersion = { .Value = FIRMWARE_VERSION };
    const Version_t gitHubVersion = { .Value = GITHUB_VERSION };
    DisplayAppInfo( "periodic-uplink-lpp",
                    &appVersion,
                    &gitHubVersion );
    BoardPrintUUID();
    BoardPrintSID();
    // Initialize transmission periodicity variable
    TxPeriodicity = gLoraAppTxDutyCycle + randr( -gLoraAppTxDutyCycleRnd, gLoraAppTxDutyCycleRnd );
    LmHandlerParams =   {
        .Region = (LoRaMacRegion_t)(uint8_t)gActiveRegion,
        .AdrEnable = (bool)gLoraAdrState,
        .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
        .TxDatarate = (int8_t)gLoraDefaultDatarate,
        .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
        .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
        .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
        .DataBuffer = AppDataBuffer,
        .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
    };

    BoardInitPeriph(); // init lora hardware in the task context (drivers modified to use os timers)

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        printf( "LoRaMac wasn't properly initialized\n" );
        Eeprom::erase();
        NvmDataMgmtFactoryReset( );
        BoardResetMcu();
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
          CliProcess( stdin );
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
//#ifndef DEBUG
      //        BoardLowPowerHandler( );
//#endif
          }
          CRITICAL_SECTION_END( );
  }
  /* USER CODE END StartTaskLeakMeter */
}


const osThreadAttr_t thread_attr = {
  .name = "default",
  .stack_size = 2048                            // Create the thread stack with a size of 1024 bytes
};
/*!
 * Main application entry point.
 */
int main( void )
{
	//Not needed but for compatibility
    BoardInitMcu( );

    osKernelInitialize();   // pre initialize CMSIS-RTOS
    MessageBus& mbus = InitOneWire();
    InitModBus(mbus);

    gLoraNode = &InitLoraNode(mbus);
    defaultTaskHandle = osThreadNew(StartTaskDefault, &mbus, &thread_attr);
    assert(defaultTaskHandle);

    // create semaphores
    gUplinkSem = osSemaphoreNew(1, 0, nullptr);
    assert(gUplinkSem);

	DBG("Heap %i\n", xPortGetFreeHeapSize());
	if (osKernelGetState() == osKernelReady) {
		osKernelStart();
	}
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
    LED_SWITCH_ON(LedsType::LED2);
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
            osTimerStart( LedBeaconTimer, LED_BEACON_TIMER_TIMEOUT );
            break;
        }
        case LORAMAC_HANDLER_BEACON_LOST:
        case LORAMAC_HANDLER_BEACON_NRX:
        {
            osTimerStop( LedBeaconTimer );
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
	static bool NewDataAvailable = false;
    if( LmHandlerIsBusy( ) == true )
    {
    	DBG("busy\n");
        return;
    }

    AppData.Port = LORAWAN_APP_PORT;

    NewDataAvailable =!NewDataAvailable? osSemaphoreAcquire(gLoraNode->AppDataChanged(), 10) == osOK : NewDataAvailable;
    if( NewDataAvailable && (LmHandlerSend( &AppData, LmHandlerParams.IsTxConfirmed ) == LORAMAC_HANDLER_SUCCESS ))
    {
    	NewDataAvailable = false;
    	gLoraNode->DataSend();
    	 DBG("DATA SENT\n");
        // Switch LED 1 ON
    	LED_SWITCH_ON(LedsType::LED1);
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
           TxTimer = osTimerNew( OnTxTimerEvent, osTimerOnce, NULL, NULL  );
           osTimerStart(TxTimer, TxPeriodicity);
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
    if(osSemaphoreAcquire(gUplinkSem, 50) == osOK)
    {
        PrepareTxFrame( );
    }
}

static void OnTxPeriodicityChanged( uint32_t periodicity )
{
    TxPeriodicity = periodicity;

    if( TxPeriodicity == 0 )
    { // Revert to application default periodicity
       TxPeriodicity = gLoraAppTxDutyCycle + randr( -gLoraAppTxDutyCycleRnd, gLoraAppTxDutyCycleRnd );
    }
    DBG("%s\n", __FUNCTION__);
    // Update timer periodicity
    osTimerStop(TxTimer );
    osTimerStart(TxTimer, TxPeriodicity);
}

static void OnTxFrameCtrlChanged( LmHandlerMsgTypes_t isTxConfirmed )
{
	DBG("%s\n", __FUNCTION__);
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
    osTimerStop(TxTimer );

    osSemaphoreRelease(gUplinkSem);//release semaphore as they created as available

    // Schedule next transmission
    osTimerStart( TxTimer, TxPeriodicity);
}

/*!
 * Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent(void* context )
{
	// Switch LED 1 OFF
    LED_SWITCH_OFF(LedsType::LED1);
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent(void* context )
{
    // Switch LED 2 OFF
    LED_SWITCH_OFF(LedsType::LED2);
}

/*!
 * Function executed on Led 2 Timeout event
 */
static void OnLed3TimerEvent(void* context )
{
    // Switch LED 2 OFF
    LED_SWITCH_OFF(LedsType::LED3);
}
/*!
 * \brief Function executed on Beacon timer Timeout event
 */
static void OnLedBeaconTimerEvent(void* context )
{
	// Switch LED 2 ON
    LED_SWITCH_ON(LedsType::LED2);
    osTimerStart( LedBeaconTimer, LED_BEACON_TIMER_TIMEOUT );
}
