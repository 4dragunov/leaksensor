/*!
 * \file      board.c
 *
 * \brief     Target board general functions implementation
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
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <assert.h>
#include "stm32f1xx.h"
#include "utilities.h"
#include "gpio.h"
#include "adc.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "timer.h"
#include "sysIrqHandlers.h"
#include "board-config.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "cmsis_os.h"
#include "OneWire.h"
#include "Ds18B20.h"
#include "FreeRTOS.h"

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    #include "sx126x-board.h"
#elif defined( LR1110MB1XXS )
    #include "lr1110-board.h"
#elif defined( SX1272MB2DAS)
    #include "sx1272-board.h"
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    #include "sx1276-board.h"
#endif
#include "board.h"


/*!
 * Unique Devices IDs register set ( STM32F103xE )
 */

typedef struct USID{
	uint32_t LotNumber:24;
	uint8_t  SiliconWaferNumber;
	uint32_t AnotherLotNumber;
	uint32_t UniqueID;
}Usid;

#define U_ID          0x1ffff7e8

#define ID1 ((uint32_t*)(U_ID + 0x0))
#define ID2 ((uint32_t*)(U_ID + 0x4))
#define ID3 ((uint32_t*)(U_ID + 0x14))


#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))
/*!
 * LED GPIO pins objects
 */
Gpio_t Led1;
Gpio_t Led2;

/*
 * MCU objects
 */
Adc_t  AdcVref = {.inst = ADC1, .channel = ADC_CHANNEL_VREFINT };
Adc_t  AdcTempSens = {.inst = ADC1, .channel = ADC_CHANNEL_TEMPSENSOR};
Uart_t Usart1;
Uart_t Usart2;

OneWire::Bus gOWI(&Usart1);
OneWire::DS18B20 gDs18b20(&gOWI, OneWire::DS18B20::Resolution::SR12BITS);

Usid *UniqueSiliconID = (Usid *) U_ID;


const ChannelConfig gChannelConfig[ADC_CHANNEL_COUNT] = {

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

ChannelPins gChannelsPins[ADC_CHANNEL_COUNT];

#if defined( LR1110MB1XXS )
    extern lr1110_t LR1110;
#endif

/*!
 * Initializes the unused GPIO to a know status
 */
static void BoardUnusedIoInit( void );

/*!
 * System Clock Configuration
 */
static void SystemClockConfig( void );

/*!
 * System Clock Re-Configuration when waking up from STOP mode
 */
static void SystemClockReConfig( void );

extern "C" void initialise_monitor_handles(void);

void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

/*!
 * UART2 FIFO buffers size
 */
#define UART1_FIFO_TX_SIZE                                1024
#define UART1_FIFO_RX_SIZE                                1024

#define UART2_FIFO_TX_SIZE                                1024
#define UART2_FIFO_RX_SIZE                                1024

uint8_t Uart1TxBuffer[UART1_FIFO_TX_SIZE];
uint8_t Uart1RxBuffer[UART1_FIFO_RX_SIZE];

uint8_t Uart2TxBuffer[UART2_FIFO_TX_SIZE];
uint8_t Uart2RxBuffer[UART2_FIFO_RX_SIZE];

void BoardCriticalSectionBegin( UBaseType_t *mask )
{
    *mask = taskENTER_CRITICAL_FROM_ISR();
}

void BoardCriticalSectionEnd( UBaseType_t *mask )
{
    taskEXIT_CRITICAL_FROM_ISR(*mask);
}

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {

        HAL_Init( );

    	SystemClockConfig( );
#ifdef DEBUG
    	initialise_monitor_handles();
#endif
        // LEDs
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

        FifoInit( &Usart1.FifoTx, Uart1TxBuffer, UART1_FIFO_TX_SIZE );
        FifoInit( &Usart1.FifoRx, Uart1RxBuffer, UART1_FIFO_RX_SIZE );

        FifoInit( &Usart2.FifoTx, Uart2TxBuffer, UART2_FIFO_TX_SIZE );
        FifoInit( &Usart2.FifoRx, Uart2RxBuffer, UART2_FIFO_RX_SIZE );
        // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        UartInit( &Usart2, USART_2, RS485_TX, RS485_RX );
        UartConfig( &Usart2, RX_TX, FIFO, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        UartInit( &Usart1, USART_1, OW_TX, OW_RX );
        UartConfig( &Usart1, RX_TX, SYNC, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        RtcInit( );

        BoardUnusedIoInit( );
        if( GetBoardPowerSource( ) == BATTERY_POWER )
        {
            // Disables OFF mode - Enables lowest power mode (STOP)
            LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
        }
    }
    else
    {
        SystemClockReConfig( );
    }

	for(int i = 0; i < ADC_CHANNEL_COUNT; i++) {

		GpioInit( &gChannelsPins[i].ptp, gChannelConfig[i].toggle_pin1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &gChannelsPins[i].ntp, gChannelConfig[i].toggle_pin2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		AdcInit( gChannelConfig[i].hadc, &gChannelsPins[i].ap, gChannelConfig[i].analog_pin, gChannelConfig[i].adc_channel);  // Just initialize ADC
	}
	printf("\n\nCore=%li, %li MHz\n", SystemCoreClock, SystemCoreClock / 1000000);
	printf("HCLK=%li\n", HAL_RCC_GetHCLKFreq());
	printf("APB1=%li\n", HAL_RCC_GetPCLK1Freq());
	printf("APB2=%li\n", HAL_RCC_GetPCLK2Freq());

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    SpiInit( &SX126x.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
    SX126xIoInit( );
#elif defined( LR1110MB1XXS )
    SpiInit( &LR1110.spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
    lr1110_board_init_io( &LR1110 );
#elif defined( SX1272MB2DAS )
    SpiInit( &SX1272.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
    SX1272IoInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiInit( &SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
    SX1276IoInit( );
#endif

    if( McuInitialized == false )
    {
        McuInitialized = true;
#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
        SX126xIoDbgInit( );
        // WARNING: If necessary the TCXO control is initialized by SX126xInit function.
#elif defined( LR1110MB1XXS )
        lr1110_board_init_dbg_io( &LR1110 );
        // WARNING: If necessary the TCXO control is initialized by SX126xInit function.
#elif defined( SX1272MB2DAS )
        SX1272IoDbgInit( );
        SX1272IoTcxoInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
        SX1276IoDbgInit( );
        SX1276IoTcxoInit( );
#endif
    }
}


static TimerEvent_t TestTimer;
static volatile bool TestTimerPassed = false;
const uint16_t times[] = { 1011, 2212}; // 10, 2, 1,
static void TestTimerEvent(void* ctx)
{
  TimerStop( &TestTimer );
  TestTimerPassed = true;
}
bool Board_Timer_Test(void){
	uint8_t pass = 0;

	TimerInit( &TestTimer, TestTimerEvent );
	for(uint8_t i = 0; i < ARRAY_SIZE(times); i++) {
		DBG("TEST TM:%i\n",times[i]);
		TimerSetValue( &TestTimer, times[i] );
		uint32_t timeStart = HAL_GetTick();
		TimerStart( &TestTimer );
		while( TestTimerPassed == false )
		{
			osDelay(1);
		}
		TestTimerPassed = false;
		uint32_t t =  HAL_GetTick() - timeStart;
		if((t>times[i] && t-times[i]<5)||(t<times[i] && times[i]-t<5)||(t==times[i])){
			DBG("TM:%i pass: %li diff: %li\n",times[i], t, t-times[i]);
			pass++;
		}else{
			DBG("TM:%i fail: %li diff: %li\n",times[i], t,  t-times[i]);
		}
	}
	return pass == ARRAY_SIZE(times);
}


void BoardResetMcu( void )
{
    CRITICAL_SECTION_BEGIN( );

    //Restart system
    NVIC_SystemReset( );
}

void BoardDeInitMcu( void )
{
    AdcDeInit( &AdcVref );
    AdcDeInit( &AdcTempSens );
    for(int i = 0; i < ADC_CHANNEL_COUNT; i++)
    	AdcDeInit( &gChannelsPins[i].ap );

#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    SpiDeInit( &SX126x.Spi );
    SX126xIoDeInit( );
#elif defined( LR1110MB1XXS )
    SpiDeInit( &LR1110.spi );
    lr1110_board_deinit_io( &LR1110 );
#elif defined( SX1272MB2DAS )
    SpiDeInit( &SX1272.Spi );
    SX1272IoDeInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
#endif
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}

void configureTimerForRunTimeStats(void){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

unsigned long getRunTimeCounterValue(void)
{
	return DWT->CYCCNT;
}

/*!
 * Factory power supply
 */
#define VDDA_VREFINT_CAL ( ( uint32_t ) 3000 )  // mV

/*!
 * VREF calibration value
 */
#define VREFINT_CAL ( *( uint16_t* ) ( ( uint32_t ) 0x1FF800F8 ) )

/*
 * Internal temperature sensor, parameter TS_CAL1: TS ADC raw data acquired at
 * a temperature of 110 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV).
 */
#define TEMP30_CAL_ADDR ( *( uint16_t* ) ( ( uint32_t ) 0x1FF8007A ) )

/* Internal temperature sensor, parameter TS_CAL2: TS ADC raw data acquired at
 *a temperature of  30 DegC (+-5 DegC), VDDA = 3.3 V (+-10 mV). */
#define TEMP110_CAL_ADDR ( *( uint16_t* ) ( ( uint32_t ) 0x1FF8007E ) )

/* Vdda value with which temperature sensor has been calibrated in production
   (+-10 mV). */
#define VDDA_TEMP_CAL ( ( uint32_t ) 3000 )

/*!
 * Battery thresholds
 */
#define BATTERY_MAX_LEVEL 3000       // mV
#define BATTERY_MIN_LEVEL 2400       // mV
#define BATTERY_SHUTDOWN_LEVEL 2300  // mV

#define BATTERY_LORAWAN_UNKNOWN_LEVEL 255
#define BATTERY_LORAWAN_MAX_LEVEL 254
#define BATTERY_LORAWAN_MIN_LEVEL 1
#define BATTERY_LORAWAN_EXT_PWR 0

#define COMPUTE_TEMPERATURE( TS_ADC_DATA, VDDA_APPLI )                                                          \
    ( ( ( ( ( ( ( int32_t )( ( TS_ADC_DATA * VDDA_APPLI ) / VDDA_TEMP_CAL ) - ( int32_t ) TEMP30_CAL_ADDR ) ) * \
            ( int32_t )( 110 - 30 ) )                                                                           \
          << 8 ) /                                                                                              \
        ( int32_t )( TEMP110_CAL_ADDR - TEMP30_CAL_ADDR ) ) +                                                   \
      ( 30 << 8 ) )

static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

uint16_t BoardBatteryMeasureVoltage( void )
{
    uint16_t vref = 0;

    // Read the current Voltage
    vref = AdcReadChannel( &AdcVref );

    // Compute and return the Voltage in millivolt
    return ( ( ( uint32_t ) VDDA_VREFINT_CAL * VREFINT_CAL ) / vref );
}

uint32_t BoardGetBatteryVoltage( void )
{
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    if( GetBoardPowerSource( ) == USB_POWER )
    {
        batteryLevel = BATTERY_LORAWAN_EXT_PWR;
    }
    else
    {
        if( BatteryVoltage >= BATTERY_MAX_LEVEL )
        {
            batteryLevel = BATTERY_LORAWAN_MAX_LEVEL;
        }
        else if( ( BatteryVoltage > BATTERY_MIN_LEVEL ) && ( BatteryVoltage < BATTERY_MAX_LEVEL ) )
        {
            batteryLevel =
                ( ( 253 * ( BatteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
        }
        else if( ( BatteryVoltage > BATTERY_SHUTDOWN_LEVEL ) && ( BatteryVoltage <= BATTERY_MIN_LEVEL ) )
        {
            batteryLevel = 1;
        }
        else  // if( BatteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
        {
            batteryLevel = BATTERY_LORAWAN_UNKNOWN_LEVEL;
        }
    }
    return batteryLevel;
}

float BoardGetInternalTemperature( void )
{
    uint16_t tempRaw = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    tempRaw = AdcReadChannel( &AdcTempSens );

    // Compute and return the temperature in degree celcius * 256
    return ( int16_t ) COMPUTE_TEMPERATURE( tempRaw, BatteryVoltage );
}

int16_t BoardGetTemperature( void ) {
	 int16_t tempRaw = NO_DATA;
     if (gDs18b20.isTempReady(0)) {
           gDs18b20.getTempRaw(0, &tempRaw);
     }else{
    	 gDs18b20.startMeasure(to_underlying(OneWire::DS18B20::Command::MEASUREALL));
     }
     return tempRaw;
}

static void BoardUnusedIoInit( void )
{
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
}

void SystemClockConfig( void )
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    __HAL_RCC_PWR_CLK_ENABLE( );


    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON; // Enabled for iwdt
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL12;
    if( HAL_RCC_OscConfig( &RCC_OscInitStruct ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if( HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_2 ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
    if( HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit ) != HAL_OK )
    {
        assert_param( LMN_STATUS_ERROR );
    }

    HAL_SYSTICK_Config( HAL_RCC_GetHCLKFreq( ) / 1000 );

    HAL_SYSTICK_CLKSourceConfig( SYSTICK_CLKSOURCE_HCLK );

    // SysTick_IRQn interrupt configuration
    HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
    SystemCoreClockUpdate();
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );

    // Enable HSI
    __HAL_RCC_HSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    // Enable PLL
    __HAL_RCC_PLL_ENABLE( );

    // Wait till PLL is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    // Select PLL as system clock source
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    // Wait till PLL is used as system clock source
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
    SystemCoreClockUpdate();
}

void SysTick_Handler( void )
{
    HAL_IncTick( );
    HAL_SYSTICK_IRQHandler( );
    osSystickHandler();
}

void HAL_MspInit(void)
{

	  /* USER CODE BEGIN MspInit 0 */

	  /* USER CODE END MspInit 0 */

	  __HAL_RCC_AFIO_CLK_ENABLE();
	  __HAL_RCC_PWR_CLK_ENABLE();

	  /* System interrupt init*/
	  /* PendSV_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

	  /* Peripheral interrupt init */
	  /* FLASH_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(FLASH_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(FLASH_IRQn);
	  /* RCC_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(RCC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(RCC_IRQn);

	  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
	  */
	  __HAL_AFIO_REMAP_SWJ_NOJTAG();

	  /* USER CODE BEGIN MspInit 1 */

	  /* USER CODE END MspInit 1 */
}

void HAL_MspDeInit(void)
{

}

uint8_t GetBoardPowerSource( void )
{
   return BATTERY_POWER;
}

/**
  * \brief Enters Low Power Stop Mode
  *
  * \note ARM exists the function when waking up
  */
void LpmEnterStopMode( void)
{

    BoardDeInitMcu( );

    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD( );

    // Clear wake up flag
    SET_BIT( PWR->CR, PWR_CR_CWUF );

    __HAL_RCC_PWR_CLK_ENABLE();
    // Enter Stop Mode
    HAL_SuspendTick();
    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
    HAL_ResumeTick();
}

/*!
 * \brief Exists Low Power Stop Mode
 */
void LpmExitStopMode( void )
{
    // Disable IRQ while the MCU is not running on HSI
    CRITICAL_SECTION_BEGIN( );
    // Initilizes the peripherals
    BoardInitMcu( );

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Enters Low Power Sleep Mode
 *
 * \note ARM exits the function when waking up
 */
void LpmEnterSleepMode( void)
{
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void BoardLowPowerHandler( void )
{
    __disable_irq( );
    /*!
     * If an interrupt has occurred after __disable_irq( ), it is kept pending 
     * and cortex will not enter low power anyway
     */

    LpmEnterLowPower( );

    __enable_irq( );
}

#ifdef USE_FULL_ASSERT

#include <stdio.h>

/*
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 */
void assert_failed( uint8_t* file, uint32_t line )
{
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %lu\n", file, line) */

    printf( "Wrong parameters value: file %s on line %lu\n", ( const char* )file, line );
    /* Infinite loop */
    while( 1 )
    {
    }
}
#endif
