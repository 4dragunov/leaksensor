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
 * @file GNSE_radio.h
 *
 * @copyright Copyright (c) 2021 The Things Industries B.V.
 *
 */

#ifndef GNSE_BSP_RADIO_H
#define GNSE_BSP_RADIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wlxx_hal.h"

extern SUBGHZ_HandleTypeDef hsubghz; // `hsubghz` is used by radio_driver.c (can't rename!)

typedef enum
{
RBI_SWITCH_OFF    = 0,
RBI_SWITCH_RX     = 1,
RBI_SWITCH_RFO_LP = 2,
RBI_SWITCH_RFO_HP = 3,

} RBI_Switch_TypeDef;

typedef enum
{
	RADIO_RFO_LP_MAXPOWER = 0,
	RADIO_RFO_HP_MAXPOWER,
} HAL_RADIO_RFOMaxPowerConfig_TypeDef;

typedef enum
{
	RBI_RFO_LP_MAXPOWER = RADIO_RFO_LP_MAXPOWER,
	RBI_RFO_HP_MAXPOWER = RADIO_RFO_HP_MAXPOWER,
} RBI_RFOMaxPowerConfig_TypeDef;

#define RBI_CONF_RFO_LP_HP  0
#define RBI_CONF_RFO_LP     1
#define RBI_CONF_RFO_HP     2

/* Indicates the type of switch between the ones proposed by CONFIG Constants
 */
#define RBI_CONF_RFO                        RBI_CONF_RFO_LP

/* Radio maximum wakeup time (in ms) */
#define RF_WAKEUP_TIME                     10U

/* Indicates whether or not TCXO is supported by the board
 * 0: TCXO not supported
 * 1: TCXO supported
 */
#define IS_TCXO_SUPPORTED                   1U

/* Indicates whether or not DCDC is supported by the board
 * 0: DCDC not supported
 * 1: DCDC supported
 */
#define IS_DCDC_SUPPORTED                   1U
//RF ANT switch power and power for the othe rf components (switch like NX3L1T3157GMZ if available for cmmutation of the rf power modes)
#define RF_SW_CTRL3_PIN                          GPIO_PIN_11
#define RF_SW_CTRL3_GPIO_PORT                    GPIOA
#define RF_SW_CTRL3_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOA_CLK_ENABLE()
#define RF_SW_CTRL3_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOA_CLK_DISABLE()

//RF ANT switch control V1  - inverted by the transistor for V2
//1 means RX and 0 means TX
#define RF_SW_CTRL2_PIN                          GPIO_PIN_13
#define RF_SW_CTRL2_GPIO_PORT                    GPIOC
#define RF_SW_CTRL2_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOC_CLK_ENABLE()
#define RF_SW_CTRL2_GPIO_CLK_DISABLE()           __HAL_RCC_GPIOC_CLK_DISABLE()
/*
//RF ANT switch control - not used in current board
#define RF_SW_CTRL1_PIN                          GPIO_PIN_12
#define RF_SW_CTRL1_GPIO_PORT                    GPIOB
#define RF_SW_CTRL1_GPIO_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define RF_SW_RX_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()
*/
#if defined(RF_CLOCK_SOURCE_TXCO)
/* A verifier car le TCXO est genere par la clock config */
#define RF_TCXO_VCC_PIN                          GPIO_PIN_0
#define RF_TCXO_VCC_GPIO_PORT                    GPIOB
#define RF_TCXO_VCC_CLK_ENABLE()                 __HAL_RCC_GPIOB_CLK_ENABLE()
#define RF_TCXO_VCC_CLK_DISABLE()                __HAL_RCC_GPIOB_CLK_DISABLE()
#elif defined(RF_CLOCK_SOURCE_XTAL)
#define HSE_VALUE 32000000
#endif

int32_t BSP_SUBGHZ_Init(void);
int32_t RBI_Init(void);
int32_t RBI_DeInit(void);
int32_t RBI_ConfigRFSwitch(RBI_Switch_TypeDef Config);
int32_t RBI_GetRFOMaxPowerConfig(RBI_RFOMaxPowerConfig_TypeDef Config);
int32_t RBI_GetTxConfig(void);
int32_t RBI_GetWakeUpTime(void);
int32_t RBI_IsTCXO(void);
int32_t RBI_IsDCDC(void);


#ifdef __cplusplus
}
#endif

#endif /* GNSE_BSP_RADIO_H */
