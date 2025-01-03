/*!
 * \file      rtc-board.c
 *
 * \brief     Target board RTC timer and low power modes management
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
 *              (C)2013-2017 Semtech - STMicroelectronics
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 *
 * \author    MCD Application Team (C)( STMicroelectronics International )
 */
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "stm32f1xx.h"
#include "utilities.h"
#include "delay.h"
#include "board.h"
#include "timer.h"
#include "systime.h"
#include "gpio.h"
#include "sysIrqHandlers.h"
#include "lpm-board.h"
#include "rtc-board.h"
#include "FreeRtos.h"
#include "task.h"
#include <stdio.h>

// MCU Wake Up Time
#define MIN_ALARM_DELAY                             1 // in msec
#define MIN_SLEEP_DELAY								1000

// RTC Time base in us
#define USEC_NUMBER                                 1000000
#define MSEC_NUMBER                                 ( USEC_NUMBER / 1000 )

#define COMMON_FACTOR                               3
#define CONV_NUMER                                  ( MSEC_NUMBER >> COMMON_FACTOR )
#define CONV_DENOM                                  ( 1 << ( N_PREDIV_S - COMMON_FACTOR ) )

/*!
 * \brief Days, Hours, Minutes and seconds
 */
#define DAYS_IN_LEAP_YEAR                           ( ( uint32_t )  366U )
#define DAYS_IN_YEAR                                ( ( uint32_t )  365U )
#define SECONDS_IN_1DAY                             ( ( uint32_t )86400U )
#define SECONDS_IN_1HOUR                            ( ( uint32_t ) 3600U )
#define SECONDS_IN_1MINUTE                          ( ( uint32_t )   60U )
#define MINUTES_IN_1HOUR                            ( ( uint32_t )   60U )
#define HOURS_IN_1DAY                               ( ( uint32_t )   24U )

/*!
 * \brief Correction factors
 */
#define  DAYS_IN_MONTH_CORRECTION_NORM              ( ( uint32_t )0x99AAA0 )
#define  DAYS_IN_MONTH_CORRECTION_LEAP              ( ( uint32_t )0x445550 )

#define COUNTER_TIME(stime)  ((uint32_t)(((uint32_t)stime.Hours * 3600U) + \
                               ((uint32_t)stime.Minutes * 60U) + \
                               ((uint32_t)stime.Seconds)))
/*!
 * \brief Calculates ceiling( X / N )
 */
#define DIVC( X, N )                                ( ( ( X ) + ( N ) -1 ) / ( N ) )



/*!
 * \brief Indicates if the RTC is already Initialized or not
 */
static bool RtcInitialized = false;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * \brief RTC Handle
 */
static RTC_HandleTypeDef RtcHandle = 
{
    .Instance = RTC,
    .Init = 
    { 
        .AsynchPrediv = RTC_AUTO_1_SECOND,
        .OutPut = RTC_OUTPUTSOURCE_ALARM,
    },
    .Lock = HAL_UNLOCKED,
    .State = HAL_RTC_STATE_RESET
};

static TIM_HandleTypeDef htim;

/*!
 * \brief RTC Alarm
 */
static RTC_AlarmTypeDef RtcAlarm;

/*!
 * Keep the value of the RTC timer when the RTC alarm is set
 * Set with the \ref RtcSetTimerContext function
 * Value is kept as a Reference to calculate alarm
 */
RtcTimerContext_t RtcTimerContext;

/*!
 * \brief Get the current time from calendar in ticks
 *
 * \param [IN] date           Pointer to RTC_DateStruct
 * \param [IN] time           Pointer to RTC_TimeStruct
 * \retval calendarValue Time in ticks
 */
static uint64_t RtcGetCalendarValue( RTC_DateTypeDef* date, RTC_TimeTypeDef* time, bool renew );
static void TimInit(void);
static void TimStartAlarm(uint32_t timer);

void RtcInit( void )
{
    RTC_DateTypeDef date;
    RTC_TimeTypeDef time;

    if( RtcInitialized == false )
    {
    	HAL_PWR_EnableBkUpAccess();
    	__HAL_RCC_PWR_CLK_ENABLE();
        __HAL_RCC_RTC_ENABLE( );

        if (HAL_RTCEx_BKUPRead( &RtcHandle, RTC_BKP_DR1 ) != 0xAA55) {
			RtcHandle.Instance            = RTC;
			RtcHandle.Init.AsynchPrediv   = RTC_AUTO_1_SECOND;  // RTC_ASYNCH_PREDIV;
			RtcHandle.Init.OutPut         = RTC_OUTPUTSOURCE_ALARM;
			HAL_RTC_Init( &RtcHandle );

			date.Year                     = 0;
			date.Month                    = RTC_MONTH_JANUARY;
			date.Date                     = 1;
			date.WeekDay                  = RTC_WEEKDAY_MONDAY;
			HAL_RTC_SetDate( &RtcHandle, &date, RTC_FORMAT_BIN );

			/*at 0:0:0*/
			time.Hours                    = 0;
			time.Minutes                  = 0;
			time.Seconds                  = 0;
			HAL_RTC_SetTime( &RtcHandle, &time, RTC_FORMAT_BIN );
			HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR1, 0xAA55 );
        }

        HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );

        HAL_NVIC_SetPriority(RTC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
        HAL_NVIC_EnableIRQ(RTC_IRQn);
        HAL_NVIC_SetPriority( RTC_Alarm_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0 );
        HAL_NVIC_EnableIRQ( RTC_Alarm_IRQn );
        __HAL_RTC_SECOND_ENABLE_IT(&RtcHandle, RTC_IT_SEC);
        // Init alarm.
        TimInit();
        RtcSetTimerContext( );

        RtcInitialized = true;
    }
}

static void TimInit(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim.Instance = TIM3;
  //https://community.st.com/t5/stm32-mcus-products/timer-tim2-goes-twice-as-fast-as-expected/td-p/531928
  htim.Init.Prescaler = (HAL_RCC_GetPCLK1Freq()/1000) - 1;
  htim.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim.Init.Period =999;
  htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim.Init.RepetitionCounter = 0;   // Only one repetition (single pulse)
  HAL_TIM_Base_Init(&htim);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig);

  HAL_TIM_OnePulse_Init(&htim, TIM_OPMODE_SINGLE);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);

  __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
  // enable timer update interrupt
  __HAL_TIM_ENABLE_IT(&htim, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM3_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

static void TimStartAlarm(uint32_t timer) {
	DBG("start tim\n");
	HAL_TIM_Base_Stop_IT(&htim);
	gettimeofday(&RtcTimerContext.Time, NULL);
	__HAL_TIM_SET_AUTORELOAD(&htim,  2*timer - 1 );
    __HAL_TIM_SET_COUNTER(&htim, 0);
    __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim);
}

void TIM3_IRQHandler()
{
	 /* TIM Update event */
	  if (__HAL_TIM_GET_FLAG(&htim, TIM_FLAG_UPDATE) != RESET)
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim, TIM_IT_UPDATE) != RESET)
	    {
	      __HAL_TIM_CLEAR_IT(&htim, TIM_IT_UPDATE);
	      DBG("stop tim\n");
	      TimerIrqHandler( );
	    }
	  }
}

/*!
 * \brief Sets the RTC timer reference, sets also the RTC_DateStruct and RTC_TimeStruct
 *
 * \param none
 * \retval timerValue In milliseconds
 */
uint32_t RtcSetTimerContext( void )
{
    gettimeofday(&RtcTimerContext.Time, 0);
    return RtcGetTimerContext();
}

/*!
 * \brief Gets the RTC timer reference
 *
 * \param none
 * \retval timerValue In milliseconds
 */
uint32_t RtcGetTimerContext( void )
{
	return ( uint32_t )RtcTimerContext.Time.tv_sec * 1000 + RtcTimerContext.Time.tv_usec / 1000;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( uint32_t milliseconds )
{
    return milliseconds / 1;
}

/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
uint32_t RtcTick2Ms( uint32_t tick )
{
    return tick * 1;
}

/*!
 * \brief a delay of delay ms by polling RTC
 *
 * \param[IN] delay in ms
 */
void RtcDelayMs( uint32_t delay )
{
    osDelay(delay);
}

/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this function) + timeout
 *
 * \param timeout Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
	struct timeval elapsed = RtcGetTimerElapsedTime();
    // We don't go in Low Power mode for timeout below MIN_ALARM_DELAY
    if( ( int64_t )MIN_SLEEP_DELAY < ( int64_t )( timeout - (elapsed.tv_sec * 1000) ) )
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );
    }
    else
    {
        LpmSetStopMode( LPM_RTC_ID, LPM_DISABLE );
    }

    if(timeout >= 1000) {
    	DBG("timer plan rtc: %li\n", timeout);
		RtcStartAlarm( timeout );
    }
    else {
    	DBG("timer plan tim: %li\n", timeout);
		TimStartAlarm( timeout );
    }
}

void RtcStopAlarm( void )
{
    // Disable the Alarm A interrupt
    HAL_RTC_DeactivateAlarm( &RtcHandle, RTC_ALARM_A );

    // Clear RTC Alarm Flag
    __HAL_RTC_ALARM_CLEAR_FLAG( &RtcHandle, RTC_FLAG_ALRAF );

    // Clear the EXTI's line Flag for RTC Alarm
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );
}

static void RtcWaitWriteCompleted(void) {

  while ((RTC->CRL & RTC_CRL_RTOFF) == 0)
    ;
}

/**
 * @brief   Acquires write access to RTC registers.
 * @details Before writing to the backup domain RTC registers the previous
 *          write operation must be completed. Use this function before
 *          writing to PRL, CNT, ALR registers.
 *
 * @notapi
 */
static bool RtcAcquireAccess(void) {
    uint32_t tickstart = 0U;
    tickstart = HAL_GetTick();
    /* Wait till RTC is in INIT state and if Time out is reached exit */
    while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
    {
      if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
      {
        return false;
      }
    }
    /* Disable the write protection for RTC registers */
    __HAL_RTC_WRITEPROTECTION_DISABLE(&RtcHandle);
    return true;
}


/**
 * @brief   Releases write access to RTC registers.
 *
 * @notapi
 */
static bool RtcReleaseAccess(void) {

	uint32_t tickstart = 0U;

	    /* Disable the write protection for RTC registers */
	__HAL_RTC_WRITEPROTECTION_ENABLE(&RtcHandle);

	tickstart = HAL_GetTick();
	    /* Wait till RTC is in INIT state and if Time out is reached exit */
	while ((RTC->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
	{
		if ((HAL_GetTick() - tickstart) >  RTC_TIMEOUT_VALUE)
		{
	        return false;
		}
	}
   return true;
}

static uint32_t RtcReadAlarmCounter(RTC_HandleTypeDef *hrtc)
{
  uint16_t high1 = 0U, low = 0U;

  high1 = READ_REG(hrtc->Instance->ALRH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->ALRL & RTC_CNTL_RTC_CNT);

  return (((uint32_t) high1 << 16U) | low);
}

static uint32_t RtcReadTimeCounter(RTC_HandleTypeDef *hrtc)
{
  uint16_t high1 = 0U, high2 = 0U, low = 0U;
  uint32_t timecounter = 0U;

  high1 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);
  low   = READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT);
  high2 = READ_REG(hrtc->Instance->CNTH & RTC_CNTH_RTC_CNT);

  if (high1 != high2)
  {
    /* In this case the counter roll over during reading of CNTL and CNTH registers,
       read again CNTL register then return the counter value */
    timecounter = (((uint32_t) high2 << 16U) | READ_REG(hrtc->Instance->CNTL & RTC_CNTL_RTC_CNT));
  }
  else
  {
    /* No counter roll over during reading of CNTL and CNTH registers, counter
       value is equal to first value of CNTL and CNTH */
    timecounter = (((uint32_t) high1 << 16U) | low);
  }

  return timecounter;
}



void RtcStartAlarm( const uint32_t timeoutMs )
{
	uint32_t seconds = timeoutMs / 1000;
	uint32_t milliseconds = timeoutMs % 1000;

    RtcStopAlarm( );

    CRITICAL_SECTION_BEGIN( );
     /* Convert time in seconds */
    if(RtcAcquireAccess()){
    	 gettimeofday(&RtcTimerContext.Time, NULL);
    	 DBG("S:%li US:%li\n", (uint32_t)RtcTimerContext.Time.tv_sec, (uint32_t)RtcTimerContext.Time.tv_usec);
		if (seconds) {
		  uint16_t realtimeoutMs = (seconds* 1000) + (1000 -  (RtcTimerContext.Time.tv_usec / 1000));
		  DBG("Real timeout before correction : %i\n", realtimeoutMs);
		  seconds += (realtimeoutMs > timeoutMs)?  -1 : 0;
		  realtimeoutMs = (seconds* 1000) + (1000 -  (RtcTimerContext.Time.tv_usec / 1000));
		  if(realtimeoutMs < 1000) {
			  //Disable STOP mode if real stop time will be  bellow 1 sec
			  LpmSetStopMode( LPM_RTC_ID, LPM_DISABLE );
		  }
		  DBG("Real timeout after correction : %i\n", realtimeoutMs);
		  //shift for current timer
		  seconds += RtcTimerContext.Time.tv_sec;
		  /* Set RTC COUNTER MSB word */
		  WRITE_REG(RTC->ALRH, ( seconds >> 16U));
			 /* Set RTC COUNTER LSB word */
		  WRITE_REG(RTC->ALRL, ( seconds & RTC_ALRL_RTC_ALR));
		  __HAL_RTC_ALARM_CLEAR_FLAG(&RtcHandle, RTC_FLAG_ALRAF);
		  /* Configure the Alarm interrupt */
		  __HAL_RTC_ALARM_ENABLE_IT(&RtcHandle, RTC_IT_ALRA);
		  /* RTC Alarm Interrupt Configuration: EXTI configuration */
		  __HAL_RTC_ALARM_EXTI_ENABLE_IT();
		  __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();
		  DBG("start rtc\n");
		}
		else {
			RTC->ALRH = 0;
			RTC->ALRL = 0;
			if(milliseconds) {
				TimStartAlarm(milliseconds);
			}
		}
		RtcReleaseAccess();
    }else {
			DBG("Error enter rtc init\n");
		}
    CRITICAL_SECTION_END( );
}

uint32_t RtcGetTimerValue( void )
{
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    uint32_t calendarValue = ( uint32_t )RtcGetCalendarValue( &date, &time, true );

    return( calendarValue );
}

struct timeval  RtcGetTimerElapsedTime( void )
{
	struct timeval elapsed ;

	struct timeval now;
	gettimeofday(&now, NULL);
	uint64_t usecs = (((now.tv_sec * 1000000) + now.tv_usec) - ((RtcTimerContext.Time.tv_sec * 1000000) + RtcTimerContext.Time.tv_usec));
	elapsed.tv_sec = usecs / 1000000;
	elapsed.tv_usec = usecs % 1000000;
  return elapsed;
}



uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
	struct timeval tv;
	struct timezone tzv;
    _gettimeofday(&tv, &tzv);
    if(milliseconds)
        *milliseconds = tv.tv_usec / 1000;
    return tv.tv_sec;
}

/*!
 * \brief RTC IRQ Handler of the RTC
 */
void RTC_IRQHandler(void)
{
	if (__HAL_RTC_SECOND_GET_IT_SOURCE(&RtcHandle, RTC_IT_SEC) &&
	    __HAL_RTC_SECOND_GET_FLAG(&RtcHandle, RTC_FLAG_SEC)) {
		if(!__HAL_RTC_SECOND_GET_FLAG(&RtcHandle, RTC_FLAG_OW))
		{
			/* Second callback */
			assert(1);
		}
		else {
			/* Second error */
			DBG("RTC overflow\n");
			/* Clear flag Second */
			__HAL_RTC_OVERFLOW_CLEAR_FLAG(&RtcHandle, RTC_FLAG_OW);
		}
		/* Clear flag Second */
		__HAL_RTC_SECOND_CLEAR_FLAG(&RtcHandle, RTC_FLAG_SEC);
	}
}

/*!
 * \brief RTC IRQ Handler of the RTC Alarm
 */
void RTC_Alarm_IRQHandler( void )
{
    RTC_HandleTypeDef* hrtc = &RtcHandle;
    CRITICAL_SECTION_BEGIN( );
    // Enable low power at irq
    LpmSetStopMode( LPM_RTC_ID, LPM_ENABLE );
    DBG("stop rtc\n");
    // Clear the EXTI's line Flag for RTC Alarm
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG( );

    // Gets the AlarmA interrupt source enable status
    if( __HAL_RTC_ALARM_GET_IT_SOURCE( hrtc, RTC_IT_ALRA ) != RESET )
    {
        // Gets the pending status of the AlarmA interrupt
        if( __HAL_RTC_ALARM_GET_FLAG( hrtc, RTC_FLAG_ALRAF ) != RESET )
        {
        	RtcStopAlarm();
            TimerIrqHandler( );
        }
    }
    CRITICAL_SECTION_END( );
}



void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
	HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR1, data0 );
    HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR2, data1 );
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
  *data0 = HAL_RTCEx_BKUPRead( &RtcHandle, RTC_BKP_DR1 );
  *data1 = HAL_RTCEx_BKUPRead( &RtcHandle, RTC_BKP_DR2 );
}

void RtcProcess( void )
{
    // Not used on this platform.
}

TimerTime_t RtcTempCompensation( TimerTime_t period, float temperature )
{
    float k = RTC_TEMP_COEFFICIENT;
    float kDev = RTC_TEMP_DEV_COEFFICIENT;
    float t = RTC_TEMP_TURNOVER;
    float tDev = RTC_TEMP_DEV_TURNOVER;
    float interim = 0.0f;
    float ppm = 0.0f;

    if( k < 0.0f )
    {
        ppm = ( k - kDev );
    }
    else
    {
        ppm = ( k + kDev );
    }
    interim = ( temperature - ( t - tDev ) );
    ppm *=  interim * interim;

    // Calculate the drift in time
    interim = ( ( float ) period * ppm ) / 1000000.0f;
    // Calculate the resulting time period
    interim += period;
    interim = floor( interim );

    if( interim < 0.0f )
    {
        interim = ( float )period;
    }

    // Calculate the resulting period
    return ( TimerTime_t ) interim;
}

#define STM32_RTCCLK 32768

int _gettimeofday(struct timeval *tp, struct timezone *tzvp)
{
	uint32_t time_frac = 0;
    if (tp) {
        /* Entering a reentrant critical zone.*/
        CRITICAL_SECTION_BEGIN( );
        //Wait for synchronization of RTC registers with APB1 bus
        while ((RTC->CRL & RTC_CRL_RSF) == 0) {};
        /* Loops until two consecutive read returning the same value.*/
         do {
        	 tp->tv_sec = ((uint32_t)(RTC->CNTH) << 16) + RTC->CNTL;
             time_frac = (((uint32_t)RTC->DIVH) << 16) + (uint32_t)RTC->DIVL;
         } while ((tp->tv_sec) != (((uint32_t)(RTC->CNTH) << 16) + RTC->CNTL));
         tp->tv_usec = ((((uint32_t)STM32_RTCCLK - 1 - time_frac) * 1000) / STM32_RTCCLK) * 1000;
         CRITICAL_SECTION_END( );
    }else{
    	assert_param(tp);
    }
    if (tzvp)  {
    	tzvp->tz_minuteswest = 0;
    	tzvp->tz_dsttime = 0;
    }else {
    	// Allowed to be not set
    	// Not a error
    }
    return 0;
}


int _settimeofday(const struct timeval *tp, const struct timezone *tzvp)
{

    if (RTC_EnterInitMode(&RtcHandle) != HAL_OK)
	{
	  return -1;
	}
	else
	{
		CRITICAL_SECTION_BEGIN( );
	    WRITE_REG(RtcHandle.Instance->CNTH, ( tp->tv_sec >> 16U));
	    WRITE_REG(RtcHandle.Instance->CNTL, ( tp->tv_sec & RTC_CNTL_RTC_CNT));
	    RTC_ExitInitMode(&RtcHandle);
	    CRITICAL_SECTION_END( );
	}
    HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR3, tzvp->tz_minuteswest );
    HAL_RTCEx_BKUPWrite( &RtcHandle, RTC_BKP_DR4, tzvp->tz_dsttime  );

    return 0;
}


