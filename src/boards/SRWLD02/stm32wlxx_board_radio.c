/** Copyright © 2021 The Things Industries B.V.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file GNSE_radio.c
 *
 * @copyright Copyright (c) 2021 The Things Industries B.V.
 *
 */

#include "stm32wlxx_board_radio.h"

SUBGHZ_HandleTypeDef hsubghz;


#if defined(RF_CLOCK_SOURCE_TXCO)
#pragma message("RF_CLOCK_SOURCE_TXCO")
static void BSP_TCXO_Init()
{
    GPIO_InitTypeDef gpio_init_structure = {0};

    RF_TCXO_VCC_CLK_ENABLE();

    gpio_init_structure.Pin = RF_TCXO_VCC_PIN;
    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(RF_TCXO_VCC_GPIO_PORT, &gpio_init_structure);

    HAL_GPIO_WritePin(RF_TCXO_VCC_GPIO_PORT, RF_TCXO_VCC_PIN, 1);
}

static void BSP_TCXO_DeInit(){
	RF_TCXO_VCC_CLK_ENABLE();
	HAL_GPIO_WritePin(RF_TCXO_VCC_GPIO_PORT, RF_TCXO_VCC_PIN, 0);
	HAL_GPIO_DeInit(RF_TCXO_VCC_GPIO_PORT, RF_TCXO_VCC_PIN);
}
#endif

#if defined(RF_CLOCK_SOURCE_XTAL)
#pragma message("RF_CLOCK_SOURCE_XTAL")
static void BSP_XTAL_Init()
{
	RCC->CR |= RCC_CR_HSEON;                            // turn on HSE (high speed external) oscillator
    while(!(RCC->CR & RCC_CR_HSERDY)){}
}

static void BSP_XTAL_DeInit(){
	HAL_GPIO_WritePin(RF_TCXO_VCC_GPIO_PORT, RF_TCXO_VCC_PIN, 0);
	HAL_GPIO_DeInit(RF_TCXO_VCC_GPIO_PORT, RF_TCXO_VCC_PIN);
}
#endif

int32_t BSP_SUBGHZ_Init(void)
{
    hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_4;
    if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
    {
        return -1;
    }
#if defined(RF_CLOCK_SOURCE_TXCO)
    BSP_TCXO_Init();
#else
    BSP_XTAL_Init();
#endif

    return 0;
}

int32_t RBI_Init(void)
{
    GPIO_InitTypeDef gpio_init_structure = {0};

    /* Enable the Radio Switch Clock */
    RF_SW_CTRL3_GPIO_CLK_ENABLE();

    /* Configure the Radio Switch pin */

    gpio_init_structure.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_structure.Pull = GPIO_NOPULL;
    gpio_init_structure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
/*  Not used in this board
    gpio_init_structure.Pin = RF_SW_CTRL1_PIN;
    HAL_GPIO_Init(RF_SW_CTRL1_GPIO_PORT, &gpio_init_structure);
*/
    gpio_init_structure.Pin = RF_SW_CTRL2_PIN;
    HAL_GPIO_Init(RF_SW_CTRL2_GPIO_PORT, &gpio_init_structure);

    gpio_init_structure.Pin = RF_SW_CTRL3_PIN;
    HAL_GPIO_Init(RF_SW_CTRL3_GPIO_PORT, &gpio_init_structure);

   // HAL_GPIO_WritePin(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_RESET);

    return 0;
}

int32_t RBI_DeInit(void)
{
#if defined(RF_CLOCK_SOURCE_TXCO)
	BSP_TCXO_DeInit();
#else
	BSP_XTAL_DeInit();
#endif
    RF_SW_CTRL3_GPIO_CLK_ENABLE();

    /* Turn off switch */
  //  HAL_GPIO_WritePin(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_RESET);

    /* DeInit the Radio Switch pin */
  //  HAL_GPIO_DeInit(RF_SW_CTRL1_GPIO_PORT, RF_SW_CTRL1_PIN);
    HAL_GPIO_DeInit(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN);
    HAL_GPIO_DeInit(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN);

    return 0;
}

int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config)
{
    switch (Config)
    {
    case RBI_SWITCH_OFF:
    {
    	//BSP_LPM_disable_lownoise_operation();
        /* Turn off switch */
        HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_RESET);
        break;
    }
    case RBI_SWITCH_RX:
    {
    	//BSP_LPM_enable_lownoise_operation();
        /*Turns On in Rx Mode the RF Switch */
        HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_SET);
        break;
    }
    case RBI_SWITCH_RFO_LP:
    {
    	//BSP_LPM_enable_lownoise_operation();
        /*Turns On in Tx Low Power the RF Switch */
        HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_RESET);
        break;
    }
    case RBI_SWITCH_RFO_HP:
    {
    	//BSP_LPM_enable_lownoise_operation();
        /*Turns On in Tx High Power the RF Switch */
        HAL_GPIO_WritePin(RF_SW_CTRL3_GPIO_PORT, RF_SW_CTRL3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(RF_SW_CTRL2_GPIO_PORT, RF_SW_CTRL2_PIN, GPIO_PIN_RESET);
        break;
    }
    default:
        break;
    }
    __HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);
    return 0;
}

int32_t RBI_GetTxConfig(void)
{
    return RBI_CONF_RFO;
}

int32_t RBI_GetWakeUpTime(void)
{
    return RF_WAKEUP_TIME;
}

int32_t RBI_IsTCXO(void)
{
    return IS_TCXO_SUPPORTED;
}

int32_t RBI_IsDCDC(void)
{
    return IS_DCDC_SUPPORTED;
}

int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config)   {
int32_t ret = 0;
/* USER CODE BEGIN RBI_GetRFOMaxPowerConfig_2 */
if (Config == RBI_RFO_LP_MAXPOWER)
{
	ret = 15; /*dBm*/
}
else
{
	ret = 22; /*dBm*/
}
/* USER CODE END RBI_GetRFOMaxPowerConfig_2 */
return ret;
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
void SUBGHZ_Radio_IRQHandler(void)
{
  HAL_SUBGHZ_IRQHandler(&hsubghz);
}

