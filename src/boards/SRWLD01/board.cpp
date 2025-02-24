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
#include <cassert>
#include <cstring>
#include <string.h>
#include <math.h>
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
#include "sensors-board.h"

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

typedef union {
	struct USID{
		uint32_t LotNumber:24;
		uint8_t  SiliconWaferNumber;
		uint32_t AnotherLotNumber;
		uint32_t UniqueID;
	} field;
	uint8_t bytes[12];
}Usid;
/*!
 * Unique Devices IDs register set ( STM32F103xE )
 */
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
Gpio_t Led3;

Gpio_t BattPwr;

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
#if defined( SX1261MBXBAS ) || defined( SX1262MBXCAS ) || defined( SX1262MBXDAS )
    SpiInit( &SX126x.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS, MASTER );
    SX126xIoInit( );
#elif defined( LR1110MB1XXS )
    SpiInit( &LR1110.spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS, MASTER );
    lr1110_board_init_io( &LR1110 );
#elif defined( SX1272MB2DAS )
    SpiInit( &SX1272.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS, MASTER );
    SX1272IoInit( );
#elif defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )
    SpiInit( &SX1276.Spi, SPI_2, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS, MASTER );
    SX1276IoInit( );
#endif

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

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {

        HAL_Init( );

    	SystemClockConfig( );
#ifdef DEBUG
    	initialise_monitor_handles();
#endif
        // LEDs OFF
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, LED_OFF );
        GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, LED_OFF );
        GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, LED_OFF );

        GpioInit( &BattPwr, BAT_PWR, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );

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
        if( GetBoardPowerSource( ) == EXT_POWER )
        {
            // Disables OFF mode - Enables lowest power mode (STOP)
            LpmSetOffMode( LPM_APPLI_ID, LPM_DISABLE );
        }
        McuInitialized = true;
    }
    else
    {
        SystemClockReConfig( );
    }
    //includes vref and ts
	for(int i = 0; i < ADC_CHANNEL_COUNT; i++) {
		GpioInit( &gChannelsPins[i].ptp, gChannelConfig[i].toggle_pin1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		GpioInit( &gChannelsPins[i].ntp, gChannelConfig[i].toggle_pin2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
		AdcInit( gChannelConfig[i].hadc, &gChannelsPins[i].ap, gChannelConfig[i].analog_pin, gChannelConfig[i].adc_channel);  // Just initialize ADC
	}
	printf("\n\nCore=%li, %li MHz\n", SystemCoreClock, SystemCoreClock / 1000000);
	printf("HCLK=%li\n", HAL_RCC_GetHCLKFreq());
	printf("APB1=%li\n", HAL_RCC_GetPCLK1Freq());
	printf("APB2=%li\n", HAL_RCC_GetPCLK2Freq());
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
    for(int i = 0; i < WL_CHANNEL_COUNT; i++)
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

uint8_t UIDtoString(const Usid *sid, char *buf, size_t bufSize)
{
  unsigned int i;
  std::strncpy(buf, (const char*)"{", bufSize);
  for(i=0;i<sizeof(Usid);i++) {
	strncat(buf, (const char*)"0x", bufSize);
    strcatNum8Hex(buf, bufSize, sid->bytes[i]);
    if (i<sizeof(Usid)-1) {
    	strncat(buf, (const char*)",", bufSize);
    }
  }
  strncat((char*)buf, (const char*)"}", bufSize);
  return 0;
}

void BoardPrintSID(void) {
	printf( "######     Silicon ID    ######\r\n");
	printf( "######   UID 0x%lx  ######\r\n", UniqueSiliconID->field.UniqueID);
	printf( "######   Wafer %i         ######\r\n", UniqueSiliconID->field.SiliconWaferNumber);
	printf( "######   Lot  %i   ######\r\n", UniqueSiliconID->field.LotNumber);
}

void BoardPrintUUID(void) {
  Usid uid;
  char buf[96];

  BoardGetUniqueId(uid.bytes);
  UIDtoString(&uid, buf, sizeof(buf));
  printf( "######   Board UUID: %s   ######\r\n\r\n", buf);
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

#define VDD_APPLI                      ((uint32_t) 3300)   /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_12BITS                   ((uint32_t) 4095)   /* Max value with a full range of 12 bits */
#define INTERNAL_TEMPSENSOR_V25        ((int32_t)1430)         /* Internal temperature sensor, parameter V25 (unit: mV). Refer to device datasheet for min/typ/max values. */
#define INTERNAL_TEMPSENSOR_AVGSLOPE   ((int32_t)4300)         /* Internal temperature sensor, parameter Avg_Slope (unit: uV/DegCelsius). Refer to device datasheet for min/typ/max values. */
/* This calibration parameter is intended to calculate the actual VDDA from Vrefint ADC measurement. */

/**
  * @brief  Computation of temperature (unit: degree Celsius) from the internal
  *         temperature sensor measurement by ADC.
  *         Computation is using temperature sensor standard parameters (refer
  *         to device datasheet).
  *         Computation formula:
  *         Temperature = (VTS - V25)/Avg_Slope + 25
  *         with VTS = temperature sensor voltage
  *              Avg_Slope = temperature sensor slope (unit: uV/DegCelsius)
  *              V25 = temperature sensor @25degC and Vdda 3.3V (unit: mV)
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value
  *            if using a different analog voltage supply value).
  * @param TS_ADC_DATA: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE_STD_PARAMS(TS_ADC_DATA)                        \
  ((((int32_t)(INTERNAL_TEMPSENSOR_V25 - (((TS_ADC_DATA) * VDD_APPLI) / RANGE_12BITS)   \
     ) * 1000                                                                  \
    ) / INTERNAL_TEMPSENSOR_AVGSLOPE                                           \
   ) + 25                                                                      \
  )

/**
  * @brief  Computation of voltage (unit: mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value
  *            if using a different analog voltage supply value).
  * @param ADC_DATA: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(ADC_DATA)  (roundf(4095.0 * 1200/(ADC_DATA)))


static uint16_t BatteryVoltage = BATTERY_MAX_LEVEL;

uint16_t BoardBatteryMeasureVoltage( void )
{
    uint16_t vref = 0;

    // Read the current Voltage
    vref = AdcReadChannel( &AdcVref );

    // Compute and return the Voltage in millivolt

    return COMPUTATION_DIGITAL_12BITS_TO_VOLTAGE(vref);
}

uint32_t BoardGetBatteryVoltage( void )
{
    return BatteryVoltage;
}

uint8_t BoardGetBatteryLevel( void )
{
    uint8_t batteryLevel = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    if( GetBoardPowerSource( ) == EXT_POWER )
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

float BoardGetTemperature( void )
{
    uint16_t tempRaw = 0;

    BatteryVoltage = BoardBatteryMeasureVoltage( );

    tempRaw = AdcReadChannel( &AdcTempSens );

    // Compute and return the temperature in degree celcius * 256
    return COMPUTATION_TEMPERATURE_STD_PARAMS( tempRaw);
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
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
  #if (INCLUDE_xTaskGetSchedulerState == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
  #endif /* INCLUDE_xTaskGetSchedulerState */
    xPortSysTickHandler();
  #if (INCLUDE_xTaskGetSchedulerState == 1 )
    }
  #endif /* INCLUDE_xTaskGetSchedulerState */
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
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

void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	__HAL_RCC_GPIOA_CLK_DISABLE( );
	__HAL_RCC_GPIOB_CLK_DISABLE( );
	__HAL_RCC_GPIOC_CLK_DISABLE( );
	__HAL_RCC_GPIOD_CLK_DISABLE( );
	__HAL_RCC_GPIOE_CLK_DISABLE( );
	__HAL_RCC_GPIOF_CLK_DISABLE( );
	__HAL_RCC_GPIOG_CLK_DISABLE( );
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	 __HAL_RCC_GPIOA_CLK_ENABLE( );
	 __HAL_RCC_GPIOB_CLK_ENABLE( );
	 __HAL_RCC_GPIOC_CLK_ENABLE( );
	 __HAL_RCC_GPIOD_CLK_ENABLE( );
	 __HAL_RCC_GPIOE_CLK_ENABLE( );
	 __HAL_RCC_GPIOF_CLK_ENABLE( );
	 __HAL_RCC_GPIOG_CLK_ENABLE( );
}

uint8_t GetBoardPowerSource( void )
{
   return (GpioRead(&BattPwr) == GPIO_PIN_SET)? EXT_POWER : BATTERY_POWER;
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
