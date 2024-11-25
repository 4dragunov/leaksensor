/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define VREF        3.3   // Опорное напряжение АЦП
#define ADC_MAX     4095   // Максимальное значение для 12-битного АЦП
#define R1          10000  // Значение известного сопротивления R1 в омах
#define UART_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

osThreadId defaultTaskHandle;
osThreadId modbusTaskHandle;
uint32_t modbusTaskBuffer[ 128 ];
osStaticThreadDef_t modbusTaskControlBlock;
osThreadId leakMeterTaskHandle;
uint32_t leakMeterTaskBuffer[ 128 ];
osStaticThreadDef_t leakMeterTaskControlBlock;
osThreadId oneWireTaskHandle;
uint32_t oneWireTaskBuffer[ 128 ];
osStaticThreadDef_t oneWireTaskControlBlock;
/* USER CODE BEGIN PV */
typedef struct {
    GPIO_TypeDef* gpio_port1;  // Указатель на порт GPIO 1 канал
    uint16_t toggle_pin1;      // Первый пин для ToggleCurrentDirection
    GPIO_TypeDef* gpio_port2;  // Указатель на порт GPIO 2 канал
    uint16_t toggle_pin2;      // Второй пин для ToggleCurrentDirection
    uint32_t adc_channel;      // Канал АЦП, используемый для данной группы
    ADC_HandleTypeDef* hadc;   // Указатель на обработчик АЦП
    uint8_t channel_number;    // Номер канала
} ChannelConfig;

ChannelConfig channels[] = {

    {GPIOE, GPIO_PIN_12, GPIOE, GPIO_PIN_1, ADC_CHANNEL_9, &hadc1, 1}, // канал 1
    {GPIOE, GPIO_PIN_13, GPIOE, GPIO_PIN_0, ADC_CHANNEL_8, &hadc1, 2}, // канал 2
    {GPIOE, GPIO_PIN_14, GPIOB, GPIO_PIN_9, ADC_CHANNEL_15, &hadc1, 3}, // канал 3
    {GPIOE, GPIO_PIN_15, GPIOB, GPIO_PIN_8, ADC_CHANNEL_14, &hadc1, 4}, // канал 4
    {GPIOB, GPIO_PIN_10, GPIOG, GPIO_PIN_14, ADC_CHANNEL_7, &hadc1, 5}, // канал 5
    {GPIOD, GPIO_PIN_9, GPIOG, GPIO_PIN_13, ADC_CHANNEL_6, &hadc1, 6}, // канал 6
    {GPIOD, GPIO_PIN_10, GPIOG, GPIO_PIN_12, ADC_CHANNEL_5, &hadc1, 7}, // канал 7
    {GPIOD, GPIO_PIN_11, GPIOG, GPIO_PIN_11, ADC_CHANNEL_4, &hadc1, 8}, // канал 8
    {GPIOD, GPIO_PIN_12, GPIOG, GPIO_PIN_10, ADC_CHANNEL_3, &hadc1, 9}, // канал 9
    {GPIOD, GPIO_PIN_13, GPIOG, GPIO_PIN_9, ADC_CHANNEL_2, &hadc1, 10}, // канал 10
    {GPIOD, GPIO_PIN_14, GPIOD, GPIO_PIN_3, ADC_CHANNEL_1, &hadc1, 11}, // канал 11
    {GPIOD, GPIO_PIN_15, GPIOD, GPIO_PIN_2, ADC_CHANNEL_13, &hadc1, 12}, // канал 12
    {GPIOG, GPIO_PIN_2, GPIOD, GPIO_PIN_1, ADC_CHANNEL_12, &hadc1, 13}, // канал 13
    {GPIOG, GPIO_PIN_3, GPIOD, GPIO_PIN_0, ADC_CHANNEL_11, &hadc1, 14}, // канал 14
    {GPIOG, GPIO_PIN_4, GPIOC, GPIO_PIN_11, ADC_CHANNEL_10, &hadc1, 15}, // канал 15
    {GPIOG, GPIO_PIN_5, GPIOC, GPIO_PIN_10, ADC_CHANNEL_8, &hadc3, 16}, // канал 16
    {GPIOG, GPIO_PIN_6, GPIOA, GPIO_PIN_12, ADC_CHANNEL_7, &hadc3, 17}, // канал 17
    {GPIOG, GPIO_PIN_7, GPIOA, GPIO_PIN_11, ADC_CHANNEL_6, &hadc3, 18}, // канал 18
    {GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_9, ADC_CHANNEL_5, &hadc3, 19}, // канал 19

    {GPIOC, GPIO_PIN_7, GPIOC, GPIO_PIN_8, ADC_CHANNEL_4, &hadc3, 20}, // канал 20
};

void ToggleCurrentDirection(GPIO_TypeDef* gpio_port1, uint16_t pin1, GPIO_TypeDef* gpio_port2, uint16_t pin2, uint16_t time_delay);
void ProcessChannel(ChannelConfig* channel);
void ConfigureUnusedChannels(ChannelConfig* active_channel);
void ResetChannelConfigs(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void StartTaskModBus(void const * argument);
void StartTaskLeakMeter(void const * argument);
void StartTaskOneWire(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *data, int len)
{
    // Отключите отладочный вывод через UART, чтобы избежать отправки лишних данных
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_CRC_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of modbusTask */
  osThreadStaticDef(modbusTask, StartTaskModBus, osPriorityNormal, 0, 128, modbusTaskBuffer, &modbusTaskControlBlock);
  modbusTaskHandle = osThreadCreate(osThread(modbusTask), NULL);

  /* definition and creation of leakMeterTask */
  osThreadStaticDef(leakMeterTask, StartTaskLeakMeter, osPriorityNormal, 0, 128, leakMeterTaskBuffer, &leakMeterTaskControlBlock);
  leakMeterTaskHandle = osThreadCreate(osThread(leakMeterTask), NULL);

  /* definition and creation of oneWireTask */
  osThreadStaticDef(oneWireTask, StartTaskOneWire, osPriorityIdle, 0, 128, oneWireTaskBuffer, &oneWireTaskControlBlock);
  oneWireTaskHandle = osThreadCreate(osThread(oneWireTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
         ProcessChannel(&channels[i]);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ESP32_nRST_Pin|LORA_IO1_Pin|LORA_IO2_Pin|LORA_IO0_Pin
                          |CH1_1_Pin|CH2_1_Pin|CH3_1_Pin|CH4_1_Pin
                          |CH2_2_Pin|CH1_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED4_Pin|CH13_1_Pin|CH14_1_Pin|CH15_1_Pin
                          |CH16_1_Pin|CH17_1_Pin|CH18_1_Pin|CH10_2_Pin
                          |CH9_2_Pin|CH8_2_Pin|CH7_2_Pin|CH6_2_Pin
                          |CH5_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH5_1_Pin|RST_LORA_Pin|CH4_2_Pin|CH3_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS_LORA_Pin|CH6_1_Pin|CH7_1_Pin|CH8_1_Pin
                          |CH9_1_Pin|CH10_1_Pin|CH11_1_Pin|CH12_1_Pin
                          |CH14_2_Pin|CH13_2_Pin|CH12_2_Pin|CH11_2_Pin
                          |DE485_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CH19_1_Pin|CH20_1_Pin|CH20_2_Pin|CH19_2_Pin
                          |CH16_2_Pin|CH15_2_Pin|OW_ST32_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CH18_2_Pin|CH17_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ESP32_nRST_Pin LORA_IO1_Pin LORA_IO2_Pin LORA_IO0_Pin
                           CH1_1_Pin CH2_1_Pin CH3_1_Pin CH4_1_Pin
                           CH2_2_Pin CH1_2_Pin */
  GPIO_InitStruct.Pin = ESP32_nRST_Pin|LORA_IO1_Pin|LORA_IO2_Pin|LORA_IO0_Pin
                          |CH1_1_Pin|CH2_1_Pin|CH3_1_Pin|CH4_1_Pin
                          |CH2_2_Pin|CH1_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : DONE_Pin */
  GPIO_InitStruct.Pin = DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin CH13_1_Pin CH14_1_Pin CH15_1_Pin
                           CH16_1_Pin CH17_1_Pin CH18_1_Pin CH10_2_Pin
                           CH9_2_Pin CH8_2_Pin CH7_2_Pin CH6_2_Pin
                           CH5_2_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|CH13_1_Pin|CH14_1_Pin|CH15_1_Pin
                          |CH16_1_Pin|CH17_1_Pin|CH18_1_Pin|CH10_2_Pin
                          |CH9_2_Pin|CH8_2_Pin|CH7_2_Pin|CH6_2_Pin
                          |CH5_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : CH5_1_Pin RST_LORA_Pin CH4_2_Pin CH3_2_Pin */
  GPIO_InitStruct.Pin = CH5_1_Pin|RST_LORA_Pin|CH4_2_Pin|CH3_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_LORA_Pin CH6_1_Pin CH7_1_Pin CH8_1_Pin
                           CH9_1_Pin CH10_1_Pin CH11_1_Pin CH12_1_Pin
                           CH14_2_Pin CH13_2_Pin CH12_2_Pin CH11_2_Pin
                           DE485_Pin */
  GPIO_InitStruct.Pin = CS_LORA_Pin|CH6_1_Pin|CH7_1_Pin|CH8_1_Pin
                          |CH9_1_Pin|CH10_1_Pin|CH11_1_Pin|CH12_1_Pin
                          |CH14_2_Pin|CH13_2_Pin|CH12_2_Pin|CH11_2_Pin
                          |DE485_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : CH19_1_Pin CH20_1_Pin CH20_2_Pin CH19_2_Pin
                           CH16_2_Pin CH15_2_Pin OW_ST32_Pin */
  GPIO_InitStruct.Pin = CH19_1_Pin|CH20_1_Pin|CH20_2_Pin|CH19_2_Pin
                          |CH16_2_Pin|CH15_2_Pin|OW_ST32_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : CH18_2_Pin CH17_2_Pin */
  GPIO_InitStruct.Pin = CH18_2_Pin|CH17_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SetUnusedChannelsHigh(ChannelConfig* active_channel) {
    // Перебираем все каналы и подаем высокий логический уровень на неактивные
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        if (&channels[i] != active_channel) {
            HAL_GPIO_WritePin(channels[i].gpio_port1, channels[i].toggle_pin1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(channels[i].gpio_port2, channels[i].toggle_pin2, GPIO_PIN_SET);
        }
    }
}

void ResetUnusedChannels(ChannelConfig* active_channel) {
    // Переводим все каналы обратно в низкий логический уровень
    for (int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++) {
        if (&channels[i] != active_channel) {
            HAL_GPIO_WritePin(channels[i].gpio_port1, channels[i].toggle_pin1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(channels[i].gpio_port2, channels[i].toggle_pin2, GPIO_PIN_RESET);
        }
    }
}


void ProcessChannel(ChannelConfig* channel) {
    const int num_measurements = 12;
    uint16_t adc_values[num_measurements];
    float sum = 0;
    uint16_t max_value = 0;
    uint16_t min_value = 0xFFFF; // �?нициализируем минимальное значение максимальным возможным
    char msg[UART_BUFFER_SIZE];

    ADC_ChannelConfTypeDef sConfig = {0};

    // Устанавливаем высокий логический уровень на неиспользуемые каналы
    SetUnusedChannelsHigh(channel);

    // Настройка канала
    sConfig.Channel = channel->adc_channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(channel->hadc, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    for (int i = 0; i < num_measurements; i++) {
        ToggleCurrentDirection(channel->gpio_port1, channel->toggle_pin1, channel->gpio_port2, channel->toggle_pin2, 30);
        HAL_GPIO_WritePin(channel->gpio_port2, channel->toggle_pin2, GPIO_PIN_SET); // подача высокого уровня перед АЦП
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET); //led on
        HAL_Delay(10);

        // Считывание значения АЦП
        HAL_ADC_Start(channel->hadc);
        HAL_ADC_PollForConversion(channel->hadc, HAL_MAX_DELAY);
        adc_values[i] = HAL_ADC_GetValue(channel->hadc);
        HAL_ADC_Stop(channel->hadc);

        sum += adc_values[i];
        if (adc_values[i] > max_value) {
            max_value = adc_values[i];
        }
        if (adc_values[i] < min_value) {
            min_value = adc_values[i];
        }

        HAL_GPIO_WritePin(channel->gpio_port2, channel->toggle_pin2, GPIO_PIN_RESET); // подача низкого уровня перед АЦП
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_RESET); //led off
        HAL_Delay(10);
    }

    uint16_t average = (uint16_t)round(sum / num_measurements);

    // Отправляем только JSON-данные через UART
    sprintf(msg, "{\"channel\": %d, \"average\": %u, \"max\": %u, \"min\": %u}\r\n", channel->channel_number, average, max_value, min_value);
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    printf("Sent JSON: %s\n", msg);

    // Сброс логического уровня на неиспользуемых каналах после измерения
    ResetUnusedChannels(channel);
}


void ToggleCurrentDirection(GPIO_TypeDef* gpio_port1, uint16_t pin1, GPIO_TypeDef* gpio_port2, uint16_t pin2, uint16_t time_delay) {
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(gpio_port1, pin1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_SET);

        HAL_GPIO_WritePin(gpio_port2, pin2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

        HAL_Delay(time_delay);
        HAL_GPIO_WritePin(gpio_port1, pin1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(gpio_port2, pin2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_SET);

        HAL_Delay(time_delay);
    }
    HAL_GPIO_WritePin(gpio_port1, pin1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_11, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(gpio_port2, pin2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12, GPIO_PIN_RESET);

    HAL_Delay(time_delay);
}

float calculateResistance(uint16_t adc_value) {
    float v_out = (double)adc_value / ADC_MAX * VREF;
    if (v_out == 0) {
        return 0;  // �?збегаем деления на ноль
    }
    float resistance = (R1 * ((VREF / v_out) - 1)) / 10000;
    return resistance;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskModBus */
/**
* @brief Function implementing the modbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskModBus */
void StartTaskModBus(void const * argument)
{
  /* USER CODE BEGIN StartTaskModBus */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
void StartTaskLeakMeter(void const * argument)
{
  /* USER CODE BEGIN StartTaskLeakMeter */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskOneWire */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM8) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
