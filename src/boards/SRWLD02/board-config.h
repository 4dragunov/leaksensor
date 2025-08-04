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

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */

#define BOARD_TCXO_WAKEUP_TIME                      5

/*!
 * Board MCU pins definitions
 */
// status Indication led
#define LED_1                                       PB_12

typedef enum {
	LED1 = 0,
} LedsType;

typedef enum {
	SE = 0,
	DIFF,
} AdcMode_t;

#define LED_ON  1
#define LED_OFF 0


#define MUX_ENABLE_TIMEOUT 2
#define MUX_SELECT_TIMEOUT 2
#define MUX_POL_SWITCH_TIMEOUT 4

#define ADC_OVS_HARDWARE DISABLE

//Half of channels selection 0- 9
#define EN0                                         PA_0
//Half of channels selection 10-19
#define EN1                                         PA_1

//One wire uart tx and rx
#define OW_TX                                       PA_2
#define OW_RX                                       PA_3
//RTC wakeup alarm - powers up the board from deep sleep
#define WAKEUP                                      PA_4
//Measurement channel polarity selection
#define PSEL                                      	PA_5
//Measurement channel selection
#define SCH0                                      	PA_6
#define SCH1                                      	PA_7
#define SCH2                                      	PA_8
#define SCH3                                      	PA_9
// One wire power delivery
#define OWPD                                    	PB_2
// Analog adc input channels
#define ADC_IN_P                                    PB_3
#define ADC_IN_N                                    PB_4
#define ADC_CH_P                                    ADC_CHANNEL_2
#define ADC_CH_N                                    ADC_CHANNEL_3
#define ADC_MODE									SE
// Modbus
#define RS485_RE                                    PB_5
#define RS485_DE									PA_12
#define RS485_RX                                    PB_6
#define RS485_TX                                    PB_7
//Rtc clock
#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15
//TXCO power
#define VDD_TXCO                                    PB_0

//Antenna multiplexer
#define FE_CTRL2									PC_13
#define FE_CTRL3									PA_11
//Debug pins
#define SWDIO										PA_13
#define SWCLK										PA_14
//Power status - battery or mains
#define BAT_PWR										PA_15
//Battery i2c
#define I2C1_SCL                                     PB_8
#define I2C1_SDA                                     PA_10


#ifdef __cplusplus
}
#endif

#endif // __BOARD_CONFIG_H__
