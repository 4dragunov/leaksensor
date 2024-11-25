/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP32_nRST_Pin GPIO_PIN_2
#define ESP32_nRST_GPIO_Port GPIOE
#define LORA_IO1_Pin GPIO_PIN_3
#define LORA_IO1_GPIO_Port GPIOE
#define LORA_IO2_Pin GPIO_PIN_4
#define LORA_IO2_GPIO_Port GPIOE
#define LORA_IO0_Pin GPIO_PIN_6
#define LORA_IO0_GPIO_Port GPIOE
#define DONE_Pin GPIO_PIN_3
#define DONE_GPIO_Port GPIOF
#define CH20_Pin GPIO_PIN_6
#define CH20_GPIO_Port GPIOF
#define CH19_Pin GPIO_PIN_7
#define CH19_GPIO_Port GPIOF
#define CH18_Pin GPIO_PIN_8
#define CH18_GPIO_Port GPIOF
#define CH17_Pin GPIO_PIN_9
#define CH17_GPIO_Port GPIOF
#define CH16_Pin GPIO_PIN_10
#define CH16_GPIO_Port GPIOF
#define CH15_Pin GPIO_PIN_0
#define CH15_GPIO_Port GPIOC
#define CH14_Pin GPIO_PIN_1
#define CH14_GPIO_Port GPIOC
#define CH13_Pin GPIO_PIN_2
#define CH13_GPIO_Port GPIOC
#define CH12_Pin GPIO_PIN_3
#define CH12_GPIO_Port GPIOC
#define CH11_Pin GPIO_PIN_1
#define CH11_GPIO_Port GPIOA
#define CH10_Pin GPIO_PIN_2
#define CH10_GPIO_Port GPIOA
#define CH9_Pin GPIO_PIN_3
#define CH9_GPIO_Port GPIOA
#define CH8_Pin GPIO_PIN_4
#define CH8_GPIO_Port GPIOA
#define CH7_Pin GPIO_PIN_5
#define CH7_GPIO_Port GPIOA
#define CH6_Pin GPIO_PIN_6
#define CH6_GPIO_Port GPIOA
#define CH5_Pin GPIO_PIN_7
#define CH5_GPIO_Port GPIOA
#define CH4_Pin GPIO_PIN_4
#define CH4_GPIO_Port GPIOC
#define CH3_Pin GPIO_PIN_5
#define CH3_GPIO_Port GPIOC
#define CH2_Pin GPIO_PIN_0
#define CH2_GPIO_Port GPIOB
#define CH1_Pin GPIO_PIN_1
#define CH1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOF
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOF
#define LED4_Pin GPIO_PIN_0
#define LED4_GPIO_Port GPIOG
#define CH1_1_Pin GPIO_PIN_12
#define CH1_1_GPIO_Port GPIOE
#define CH2_1_Pin GPIO_PIN_13
#define CH2_1_GPIO_Port GPIOE
#define CH3_1_Pin GPIO_PIN_14
#define CH3_1_GPIO_Port GPIOE
#define CH4_1_Pin GPIO_PIN_15
#define CH4_1_GPIO_Port GPIOE
#define CH5_1_Pin GPIO_PIN_10
#define CH5_1_GPIO_Port GPIOB
#define RST_LORA_Pin GPIO_PIN_12
#define RST_LORA_GPIO_Port GPIOB
#define CS_LORA_Pin GPIO_PIN_8
#define CS_LORA_GPIO_Port GPIOD
#define CH6_1_Pin GPIO_PIN_9
#define CH6_1_GPIO_Port GPIOD
#define CH7_1_Pin GPIO_PIN_10
#define CH7_1_GPIO_Port GPIOD
#define CH8_1_Pin GPIO_PIN_11
#define CH8_1_GPIO_Port GPIOD
#define CH9_1_Pin GPIO_PIN_12
#define CH9_1_GPIO_Port GPIOD
#define CH10_1_Pin GPIO_PIN_13
#define CH10_1_GPIO_Port GPIOD
#define CH11_1_Pin GPIO_PIN_14
#define CH11_1_GPIO_Port GPIOD
#define CH12_1_Pin GPIO_PIN_15
#define CH12_1_GPIO_Port GPIOD
#define CH13_1_Pin GPIO_PIN_2
#define CH13_1_GPIO_Port GPIOG
#define CH14_1_Pin GPIO_PIN_3
#define CH14_1_GPIO_Port GPIOG
#define CH15_1_Pin GPIO_PIN_4
#define CH15_1_GPIO_Port GPIOG
#define CH16_1_Pin GPIO_PIN_5
#define CH16_1_GPIO_Port GPIOG
#define CH17_1_Pin GPIO_PIN_6
#define CH17_1_GPIO_Port GPIOG
#define CH18_1_Pin GPIO_PIN_7
#define CH18_1_GPIO_Port GPIOG
#define CH19_1_Pin GPIO_PIN_6
#define CH19_1_GPIO_Port GPIOC
#define CH20_1_Pin GPIO_PIN_7
#define CH20_1_GPIO_Port GPIOC
#define CH20_2_Pin GPIO_PIN_8
#define CH20_2_GPIO_Port GPIOC
#define CH19_2_Pin GPIO_PIN_9
#define CH19_2_GPIO_Port GPIOC
#define CH18_2_Pin GPIO_PIN_11
#define CH18_2_GPIO_Port GPIOA
#define CH17_2_Pin GPIO_PIN_12
#define CH17_2_GPIO_Port GPIOA
#define JTMS_SWDIO_Pin GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port GPIOA
#define JTCK_SWCLK_Pin GPIO_PIN_14
#define JTCK_SWCLK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define CH16_2_Pin GPIO_PIN_10
#define CH16_2_GPIO_Port GPIOC
#define CH15_2_Pin GPIO_PIN_11
#define CH15_2_GPIO_Port GPIOC
#define OW_ST32_Pin GPIO_PIN_12
#define OW_ST32_GPIO_Port GPIOC
#define CH14_2_Pin GPIO_PIN_0
#define CH14_2_GPIO_Port GPIOD
#define CH13_2_Pin GPIO_PIN_1
#define CH13_2_GPIO_Port GPIOD
#define CH12_2_Pin GPIO_PIN_2
#define CH12_2_GPIO_Port GPIOD
#define CH11_2_Pin GPIO_PIN_3
#define CH11_2_GPIO_Port GPIOD
#define DE485_Pin GPIO_PIN_4
#define DE485_GPIO_Port GPIOD
#define TX485TM_Pin GPIO_PIN_5
#define TX485TM_GPIO_Port GPIOD
#define RX485TM_Pin GPIO_PIN_6
#define RX485TM_GPIO_Port GPIOD
#define CH10_2_Pin GPIO_PIN_9
#define CH10_2_GPIO_Port GPIOG
#define CH9_2_Pin GPIO_PIN_10
#define CH9_2_GPIO_Port GPIOG
#define CH8_2_Pin GPIO_PIN_11
#define CH8_2_GPIO_Port GPIOG
#define CH7_2_Pin GPIO_PIN_12
#define CH7_2_GPIO_Port GPIOG
#define CH6_2_Pin GPIO_PIN_13
#define CH6_2_GPIO_Port GPIOG
#define CH5_2_Pin GPIO_PIN_14
#define CH5_2_GPIO_Port GPIOG
#define JTRST_Pin GPIO_PIN_4
#define JTRST_GPIO_Port GPIOB
#define OW_TX_Pin GPIO_PIN_6
#define OW_TX_GPIO_Port GPIOB
#define OW_RX_Pin GPIO_PIN_7
#define OW_RX_GPIO_Port GPIOB
#define CH4_2_Pin GPIO_PIN_8
#define CH4_2_GPIO_Port GPIOB
#define CH3_2_Pin GPIO_PIN_9
#define CH3_2_GPIO_Port GPIOB
#define CH2_2_Pin GPIO_PIN_0
#define CH2_2_GPIO_Port GPIOE
#define CH1_2_Pin GPIO_PIN_1
#define CH1_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
