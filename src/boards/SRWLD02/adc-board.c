/*!
 * \file      adc-board.c
 *
 * \brief     Target board ADC driver implementation
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
#include "stm32wlxx.h"
#include "board-config.h"
#include "adc-board.h"
#include "gpio.h"
#include  <assert.h>

ADC_HandleTypeDef hadc = {.Instance = ADC};

void AdcMcuInit( Adc_t *obj, PinNames adcInput )
{
	ADC_HandleTypeDef * h = &hadc;
	assert(h);

	 __HAL_RCC_ADC_CLK_ENABLE( );


	 HAL_ADC_Init(h);

    if( adcInput != NC )
    {
        GpioInit( &obj->AdcInput, adcInput, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    }
}

void AdcMcuDeInit( Adc_t *obj )
{
	ADC_HandleTypeDef * h = &hadc;
	assert(h);

	HAL_ADC_DeInit(h);

	__HAL_RCC_ADC_CLK_DISABLE( );
}

void AdcMcuConfig( Adc_t *obj )
{
    // Configure ADC
	assert(obj);
	assert(obj->inst);
	ADC_HandleTypeDef * h = &hadc;
	hadc.Instance = ADC;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.NbrOfConversion = 1;
	hadc.Init.DiscontinuousConvMode = ENABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;

	hadc.Init.OversamplingMode = ADC_OVS_HARDWARE;
	hadc.Init.Oversampling.Ratio = ADC_OVERSAMPLING_RATIO_16;
	hadc.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
	hadc.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
	hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_LOW;
    HAL_ADC_Init( h );
    HAL_ADCEx_Calibration_Start(h);
}

uint16_t AdcMcuReadChannel( Adc_t *obj )
{
	ADC_HandleTypeDef * h = &hadc;
    ADC_ChannelConfTypeDef adcConf = { 0 };
    uint16_t adcData = 0;

    // Enable HSI
  //  __HAL_RCC_HSI_ENABLE( );

    // Wait till HSI is ready
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSIRDY ) == RESET )
    {
    }

    __HAL_RCC_ADC_CLK_ENABLE( );


    adcConf.Channel = obj->channel;
    adcConf.Rank = ADC_REGULAR_RANK_1;
    adcConf.SamplingTime = ((obj->channel== ADC_CHANNEL_VREFINT) ||
    						(obj->channel== ADC_CHANNEL_TEMPSENSOR))? ADC_SAMPLETIME_79CYCLES_5 : ADC_SAMPLETIME_1CYCLE_5;

    HAL_ADC_ConfigChannel( h, &adcConf );

    // Start ADC Software Conversion
    HAL_ADC_Start( h );

    HAL_ADC_PollForConversion( h, HAL_MAX_DELAY );

    adcData = HAL_ADC_GetValue( h );

    __HAL_RCC_ADC_CLK_DISABLE( );

    // Disable HSI
   // __HAL_RCC_HSI_DISABLE( );

    return adcData;
}
