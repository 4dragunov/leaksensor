#include <stdint.h>
#include <stm32f1xx.h>
#include <sys/time.h>
#include <stm32f1xx_ll_pwr.h>
#include <stm32f1xx_ll_rtc.h>

#include "FreeRtos.h"
#include "task.h"
#include "utilities.h"
#include "rtc-board.h"
/*
void vPortSuppressTicksAndSleep(portTickType xExpectedIdleTime) {
	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	// systick IRQ off
	LL_PWR_ClearFlag_WU();

	portTickType xActualIdleTime = 0;
	uint32_t subSecond = RTC_GetSubSecond();
	eSleepModeStatus eSleepStatus = eTaskConfirmSleepModeStatus();
	__asm volatile ("cpsid i");
	__DSB();
	__ISB();
	if (eSleepStatus != eAbortSleep) {
		RTC_ClearFlag (RTC_FLAG_WUTF);
		// RTC Set WakeUp Counter
		RTC_SetWakeUpCounter(
				(xExpectedIdleTime
						* (LSE_FREQUENCY / RTC_WakeUpClock_RTCCLK_Div)) / 1000);
		RTC_WakeUpCmd (ENABLE);
		__DSB();
		PWR_EnterSleepMode(PWR_Regulator_LowPower, PWR_SLEEPEntry_WFI);
		__ISB();
		RTC_WakeUpCmd (DISABLE);
	}
	if (RTC_GetFlagStatus(RTC_FLAG_WUTF) == SET) {
		xActualIdleTime = xExpectedIdleTime;
	} else {
		xActualIdleTime = (RTC_GetSubSecond() - subSecond) * 8;
		if (xActualIdleTime > xExpectedIdleTime)
			xActualIdleTime = xExpectedIdleTime;
	}
	vTaskStepTick(xActualIdleTime / portTICK_RATE_MS);
	__asm volatile ("cpsie i");
	__DSB();
	__ISB();
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	// systick IRQ on
}
*/

#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )

constexpr uint64_t timeval2us(const struct timeval &lhs)
{
	return  (lhs.tv_sec *1000 * 1000) + lhs.tv_usec;
}

static inline void prvStopTickInterruptTimer(void)
{
	/* Stop the SysTick momentarily.  The time the SysTick is stopped for
	is accounted for as best it can be, but using the tickless mode will
	inevitably result in some tiny drift of the time maintained by the
	kernel with respect to calendar time. */
	portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;
}

static inline void prvStartTickInterruptTimer(void)
{
	portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
}

static inline void vSetWakeTimeInterrupt( portTickType xExpectedIdleTime )
{
	auto currenttime = RtcGetTimerValue();
	auto currentAlarm = RtcGetAlarmValue();
	if(((currentAlarm - currenttime) * 1000) > xExpectedIdleTime)
		RtcStartAlarm(xExpectedIdleTime * portTICK_RATE_MS);
}

static inline void disable_interrupts(){
	__disable_irq();
}

static inline void enable_interrupts(){
	__enable_irq();
}

static inline void prvSleep( TickType_t& xExpectedIdleTime ){
	TickType_t xModifiableIdleTime = xExpectedIdleTime;
	/* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
	set its parameter to 0 to indicate that its implementation contains
	its own wait for interrupt or wait for event instruction, and so wfi
	should not be executed again.  However, the original expected idle
	time variable must remain unmodified, so a copy is taken. */

	configPRE_SLEEP_PROCESSING( &xModifiableIdleTime );
	if( xModifiableIdleTime > 0 )
	{
		LL_PWR_ClearFlag_WU();
		__DSB();
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		__ISB();
	}
	configPOST_SLEEP_PROCESSING( &xExpectedIdleTime );
}

#if( configUSE_TICKLESS_IDLE == 1 )
/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
extern "C" void vPortSuppressTicksAndSleep( TickType_t xExpectedIdleTime )
{
struct timeval ulLowPowerTimeBeforeSleep, ulLowPowerTimeAfterSleep;
eSleepModeStatus eSleepStatus;

    /* Read the current time from a time source that will remain operational
       while the microcontroller is in a low power state. */
    gettimeofday(&ulLowPowerTimeBeforeSleep, 0);
    /* Stop the timer that is generating the tick interrupt. */
    prvStopTickInterruptTimer();

    /* Enter a critical section that will not effect interrupts bringing the MCU
       out of sleep mode. */
    disable_interrupts();

    /* Ensure it is still ok to enter the sleep mode. */
    eSleepStatus = eTaskConfirmSleepModeStatus();

    if( eSleepStatus == eAbortSleep )
    {
        /* A task has been moved out of the Blocked state since this macro was
           executed, or a context siwth is being held pending. Do not enter a
           sleep state. Restart the tick and exit the critical section. */
        prvStartTickInterruptTimer();
        enable_interrupts();
    }
    else
    {
        if( eSleepStatus == eNoTasksWaitingTimeout )
        {
            /* It is not necessary to configure an interrupt to bring the
               microcontroller out of its low power state at a fixed time in the
               future. */
            prvSleep(xExpectedIdleTime);
        }
        else
        {
            /* Configure an interrupt to bring the microcontroller out of its low
               power state at the time the kernel next needs to execute. The
               interrupt must be generated from a source that remains operational
               when the microcontroller is in a low power state. */
            vSetWakeTimeInterrupt( xExpectedIdleTime );

            /* Enter the low power state. */
            prvSleep(xExpectedIdleTime);

            /* Determine how long the microcontroller was actually in a low power
               state for, which will be less than xExpectedIdleTime if the
               microcontroller was brought out of low power mode by an interrupt
               other than that configured by the vSetWakeTimeInterrupt() call.
               Note that the scheduler is suspended before
               portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
               portSUPPRESS_TICKS_AND_SLEEP() returns. Therefore no other tasks will
               execute until this function completes. */
            gettimeofday(&ulLowPowerTimeAfterSleep, 0);

            /* Correct the kernels tick count to account for the time the
               microcontroller spent in its low power state. */
            uint64_t sleepUs = timeval2us(ulLowPowerTimeAfterSleep - ulLowPowerTimeBeforeSleep);
            vTaskStepTick(sleepUs /(portTICK_RATE_MS * 1000) );
        }

        /* Exit the critical section - it might be possible to do this immediately
           after the prvSleep() calls. */
        enable_interrupts();

        /* Restart the timer that is generating the tick interrupt. */
        prvStartTickInterruptTimer();
    }
}
#endif
