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
#include "stm32wlxx.h"
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

#include "stm32wlxx_board_radio.h"

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

Gpio_t BattPwr;
Gpio_t SensorsEn[2];
Gpio_t SensorsPolarity;

/*
 * MCU objects
 */
Adc_t  AdcVref = {.inst = ADC, .channel = ADC_CHANNEL_VREFINT };
Adc_t  AdcTempSens = {.inst = ADC, .channel = ADC_CHANNEL_TEMPSENSOR};
Adc_t  AdcInP = {.inst = ADC, .channel = ADC_CH_P};
Adc_t  AdcInN = {.inst = ADC, .channel = ADC_CH_N};

Uart_t LpUart1;
Uart_t Usart1;

OneWire::Bus gOWI(&LpUart1);
OneWire::DS18B20 gDs18b20(&gOWI, OneWire::DS18B20::Resolution::SR12BITS);

Usid *UniqueSiliconID = (Usid *) U_ID;


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
#define LPUART1_FIFO_TX_SIZE                                1024
#define LPUART1_FIFO_RX_SIZE                                1024

#define USART1_FIFO_TX_SIZE                                1024
#define USART1_FIFO_RX_SIZE                                1024

uint8_t LpUart1TxBuffer[LPUART1_FIFO_TX_SIZE];
uint8_t LpUart1RxBuffer[LPUART1_FIFO_RX_SIZE];

uint8_t Usart1TxBuffer[USART1_FIFO_TX_SIZE];
uint8_t Usart1RxBuffer[USART1_FIFO_RX_SIZE];

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
        // LEDs OFF
        GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, LED_OFF );

        GpioInit( &BattPwr, BAT_PWR, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );

        FifoInit( &LpUart1.FifoTx, LpUart1TxBuffer, LPUART1_FIFO_TX_SIZE );
        FifoInit( &LpUart1.FifoRx, LpUart1RxBuffer, LPUART1_FIFO_RX_SIZE );

        FifoInit( &Usart1.FifoTx, Usart1TxBuffer, USART1_FIFO_TX_SIZE );
        FifoInit( &Usart1.FifoRx, Usart1RxBuffer, USART1_FIFO_RX_SIZE );


        // Configure your terminal for 8 Bits data (7 data bit + 1 parity bit), no parity and no flow ctrl
        UartInit( &Usart1, USART_1, RS485_TX, RS485_RX, PIN_PUSH_PULL);
        UartConfig( &Usart1, RX_TX, FIFO, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        UartInit( &LpUart1, LPUART_1, OW_TX, OW_RX, PIN_OPEN_DRAIN );
        UartConfig( &LpUart1, RX_TX, SYNC, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );

        RtcInit( );

        BoardUnusedIoInit( );
        if( BoardGetPowerSource( ) == EXT_POWER )
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
	GpioInit( &SensorsEn[0], EN0, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &SensorsEn[1], EN1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	GpioInit( &SensorsPolarity, PSEL, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
	AdcInit( ADC, &AdcInP, ADC_IN_P, ADC_CH_P);  // Just initialize ADC
	AdcInit( ADC, &AdcInN, ADC_IN_N, ADC_CH_N);  // Just initialize ADC

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
    AdcDeInit( &AdcInP );
    AdcDeInit( &AdcInN );
}

uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
	  uint32_t val = 0;
	  val = LL_FLASH_GetUDN();
	  if (val == 0xFFFFFFFF)  /* Normally this should not happen */
	  {
	    uint32_t ID_1_3_val = HAL_GetUIDw0() + HAL_GetUIDw2();
	    uint32_t ID_2_val = HAL_GetUIDw1();

	    id[7] = (ID_1_3_val) >> 24;
	    id[6] = (ID_1_3_val) >> 16;
	    id[5] = (ID_1_3_val) >> 8;
	    id[4] = (ID_1_3_val);
	    id[3] = (ID_2_val) >> 24;
	    id[2] = (ID_2_val) >> 16;
	    id[1] = (ID_2_val) >> 8;
	    id[0] = (ID_2_val);
	  }
	  else  /* Typical use case */
	  {
	    id[7] = val & 0xFF;
	    id[6] = (val >> 8) & 0xFF;
	    id[5] = (val >> 16) & 0xFF;
	    id[4] = (val >> 24) & 0xFF;
	    val = LL_FLASH_GetDeviceID();
	    id[3] = val & 0xFF;
	    val = LL_FLASH_GetSTCompanyID();
	    id[2] = val & 0xFF;
	    id[1] = (val >> 8) & 0xFF;
	    id[0] = (val >> 16) & 0xFF;
	  }
}

uint32_t GetDevAddr(void)
{
  uint32_t val = 0;
  val = LL_FLASH_GetUDN();
  if (val == 0xFFFFFFFF)
  {
    val = ((HAL_GetUIDw0()) ^ (HAL_GetUIDw1()) ^ (HAL_GetUIDw2()));
  }
  return val;
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

    if( BoardGetPowerSource( ) == EXT_POWER )
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
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	  /** Configure LSE Drive Capability
	  */
	  HAL_PWR_EnableBkUpAccess();
	  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	  /** Configure the main internal regulator output voltage
	  */
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
	                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
	  RCC_OscInitStruct.LSIDiv = RCC_LSI_DIV1;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {

	  }

	  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
	                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
	                              |RCC_CLOCKTYPE_PCLK2;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	  {
	  }

    // SysTick_IRQn interrupt configuration
 //   HAL_NVIC_SetPriority( SysTick_IRQn, 0, 0 );
  //  SystemCoreClockUpdate();
}

void SystemClockReConfig( void )
{
  //  __HAL_RCC_PWR_CLK_ENABLE( );

    // Enable MSI
    __HAL_RCC_MSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_MSIRDY ) == RESET )
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
   // HAL_IncTick();
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


	  /* USER CODE END MspInit 0 */
	  PWR_PVDTypeDef sConfigPVD = {0};

	  __HAL_RCC_HSEM_CLK_ENABLE();

	  /* System interrupt init*/
	  /* PendSV_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

	  /* Peripheral interrupt init */
	  /* HSEM_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(HSEM_IRQn, 5, 0);
	  HAL_NVIC_EnableIRQ(HSEM_IRQn);
	  /* FLASH_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(FLASH_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(FLASH_IRQn);
	  /* RCC_IRQn interrupt configuration */
	  HAL_NVIC_SetPriority(RCC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, 0);
	  HAL_NVIC_EnableIRQ(RCC_IRQn);

	  /** PVD Configuration
	  */
	  sConfigPVD.PVDLevel = PWR_PVDLEVEL_3;
	  sConfigPVD.Mode = PWR_PVD_MODE_NORMAL;
	  HAL_PWR_ConfigPVD(&sConfigPVD);

	  /** Enable the PVD Output
	  */
	  HAL_PWR_EnablePVD();


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
	  /* Called by the kernel before it places the MCU into a sleep mode because
	  configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

	  NOTE:  Additional actions can be taken here to get the power consumption
	  even lower.  For example, peripherals can be turned off here, and then back
	  on again in the post sleep processing function.  For maximum power saving
	  ensure all unused pins are in their lowest power state. */

	  /*
	    (*ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
	    its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep
	    function does not need to execute the wfi instruction
	  */
	  *ulExpectedIdleTime = 0;

	  /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
	  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	 __HAL_RCC_GPIOA_CLK_ENABLE( );
	 __HAL_RCC_GPIOB_CLK_ENABLE( );
	 __HAL_RCC_GPIOC_CLK_ENABLE( );
	  /* Called by the kernel when the MCU exits a sleep mode because
	  configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

	  /* Avoid compiler warnings about the unused parameter. */
	  (void) ulExpectedIdleTime;
}

uint8_t BoardGetPowerSource( void )
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
   // SET_BIT( PWR->CR, PWR_CR_CWUF );

  //  __HAL_RCC_PWR_CLK_ENABLE();
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
