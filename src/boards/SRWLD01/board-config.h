/*!
 * \file      board-config.h
 *
 * \brief     Board configuration
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
 *               ___ _____ _   ___ _  _____ ___  ___  ___ ___
 *              / __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
 *              \__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
 *              |___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
 *              embedded.connectivity.solutions===============
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    Daniel Jaeckle ( STACKFORCE )
 *
 * \author    Johannes Bruder ( STACKFORCE )
 */
#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include "gpio.h"
#include "adc.h"
/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#if defined( SX1262MBXDAS )
#define BOARD_TCXO_WAKEUP_TIME                      5
#else
#define BOARD_TCXO_WAKEUP_TIME                      0
#endif

/*!
 * Board MCU pins definitions
 */
#define RADIO_RESET                                 PB_12

#define RADIO_MOSI                                  PB_15
#define RADIO_MISO                                  PB_14
#define RADIO_SCLK                                  PB_13

#if defined( SX1272MB2DAS) || defined( SX1276MB1LAS ) || defined( SX1276MB1MAS )

#define RADIO_NSS                                   PD_8

#define RADIO_DIO_0                                 PE_6
#define RADIO_DIO_1                                 PE_3
#define RADIO_DIO_2                                 PE_4

//#define RADIO_DIO_3                                 PB_4
//#define RADIO_DIO_4                                 PA_9
//#define RADIO_DIO_5                                 PC_7

//#define RADIO_ANT_SWITCH

#define LED_1                                       PF_11
#define LED_2                                       PF_12

// Debug pins definition.
//#define RADIO_DBG_PIN_TX                            PB_0
//#define RADIO_DBG_PIN_RX                            PA_4

#endif

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

//#define OSC_HSE_IN                                  PH_0
//#define OSC_HSE_OUT                                 PH_1

#define SWCLK                                       PA_14
#define SWDAT                                       PA_13

//#define I2C_SCL                                     PB_8
//#define I2C_SDA                                     PB_9
//usart2
#define RS485_TX                                     PD_5
#define RS485_RX                                     PD_6
//usart1
#define OW_RX                                    PA_10
#define OW_TX                                    PA_9


typedef struct {
	PinNames toggle_pin1;      // Первый пин для ToggleCurrentDirection
	PinNames toggle_pin2;      // Второй пин для ToggleCurrentDirection
	PinNames analog_pin;       // Вход АЦП
	uint32_t adc_channel;
	void* hadc;               // Указатель на обработчик АЦП
} ChannelConfig;

typedef struct {
	Gpio_t ptp;
	Gpio_t ntp;
	Adc_t ap;
} ChannelPins;

#define ADC_CHANNEL_COUNT 20

#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
