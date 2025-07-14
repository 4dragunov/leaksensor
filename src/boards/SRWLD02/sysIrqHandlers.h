/*!
 * \file      sysIrqHandlers.h
 *
 * \brief     Default IRQ handlers
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
 *              (C)2013-2020 Semtech
 *
 * \endcode
 */
#ifndef SYS_IRQ_HANDLERS_H
#define SYS_IRQ_HANDLERS_H

#ifdef __cplusplus
 extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void TAMP_STAMP_LSECSS_SSRU_IRQHandler(void);
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void USART2_IRQHandler(void);
void RTC_Alarm_IRQHandler(void);
void SUBGHZ_Radio_IRQHandler(void);
void TIM17_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif // SYS_IRQ_HANDLERS_H
