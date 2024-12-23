/*!
 * \file      timer.c
 *
 * \brief     Timer objects and scheduling management implementation
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
#include <assert.h>
#include "utilities.h"
#include "board.h"
#include <sys/time.h>
#include "rtc-board.h"
#include "timer.h"
#include "stdio.h"

struct TimerTime: timeval {

TimerTime& operator+(const struct timeval& b)
{
	uint64_t usecs =  ((tv_sec + b.tv_sec) * 1000000) + (tv_usec + b.tv_usec);
	tv_sec = usecs / 1000000;
	tv_usec = usecs % 1000000;
	return *this;
}

TimerTime& operator+( const uint32_t& msecs)
{
	uint64_t usecs =  (tv_sec *1000000 + tv_usec) + msecs * 1000;
	tv_sec = usecs / 1000000;
	tv_usec = usecs % 1000000;
	return *this;
}

TimerTime& operator+( const int32_t& msecs)
{
	uint64_t usecs =  (tv_sec *1000000 + tv_usec) + msecs * 1000;
	tv_sec = usecs / 1000000;
	tv_usec = usecs % 1000000;
	return *this;
}

TimerTime& operator+=(const struct timeval& b)
{
	uint32_t usecs =  (tv_usec + b.tv_usec) / 1000000;
	uint32_t usecs2 =  (tv_usec + b.tv_usec) % 1000000;
	tv_sec = tv_sec + b.tv_sec + usecs;
	tv_usec = usecs2;
	return *this;
}

bool operator<(const struct timeval& r)
{
	return (tv_sec * 1000000 +  tv_usec) < (r.tv_sec * 1000000 +  r.tv_usec);
}

bool operator==(const struct timeval& r)
{
	return (tv_sec * 1000000 +  tv_usec) == (r.tv_sec * 1000000 +  r.tv_usec);
}

operator TimerTime_t() const
{
	return (TimerTime_t)(tv_sec * 1000000 +  tv_usec)/ 1000;
}

operator struct timeval&()
{
	return *this;
}

TimerTime(const TimerTime_t&r):timeval(){
	tv_sec = r/1000;
	tv_usec = (r - (tv_sec * 1000));
}
TimerTime(const struct timeval& r):timeval(){
	tv_sec = r.tv_sec;
	tv_usec = r.tv_usec;
}
} ;
/*!
 * Safely execute call back
 */
#define ExecuteCallBack( _callback_, context ) \
    do                                         \
    {                                          \
        if( _callback_ == NULL )               \
        {                                      \
            while( 1 );                        \
        }                                      \
        else                                   \
        {                                      \
            _callback_( context );             \
        }                                      \
    }while( 0 );

/*!
 * Timers list head pointer
 */
volatile static TimerEvent_t *TimerListHead = NULL;

/*!
 * \brief Adds or replace the head timer of the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be become the new head
 * \param [IN]  remainingTime Remaining time of the previous head to be replaced
 */
static void TimerInsertNewHeadTimer( TimerEvent_t *obj );

/*!
 * \brief Adds a timer to the list.
 *
 * \remark The list is automatically sorted. The list head always contains the
 *         next timer to expire.
 *
 * \param [IN]  obj Timer object to be added to the list
 * \param [IN]  remainingTime Remaining time of the running head after which the object may be added
 */
static void TimerInsertTimer( TimerEvent_t *obj );

/*!
 * \brief Sets a timeout with the duration "timestamp"
 *
 * \param [IN] timestamp Delay duration
 */
static void TimerSetTimeout( TimerEvent_t *obj );

/*!
 * \brief Check if the Object to be added is not already in the list
 *
 * \param [IN] timestamp Delay duration
 * \retval true (the object is already in the list) or false
 */
static bool TimerExists( TimerEvent_t *obj );

void TimerInit( TimerEvent_t *obj, void ( *callback )( void *context ) )
{
    obj->Timestamp = 0;
    obj->ReloadValue = 0;
    obj->IsStarted = false;
    obj->IsNext2Expire = false;
    obj->Callback = callback;
    obj->Context = NULL;
    obj->Next = NULL;
}

void TimerSetContext( TimerEvent_t *obj, void* context )
{
    obj->Context = context;
}

void TimerStart( TimerEvent_t *obj )
{
    struct TimerTime elapsedTime = 0;

    CRITICAL_SECTION_BEGIN( );

    if( ( obj == NULL ) || ( TimerExists( obj ) == true ) )
    {
        CRITICAL_SECTION_END( );
        return;
    }

    obj->Timestamp = obj->ReloadValue;
    obj->IsStarted = true;
    obj->IsNext2Expire = false;

    if( TimerListHead == NULL )
    {
        RtcSetTimerContext( );
        // Inserts a timer at time now + obj->Timestamp
        TimerInsertNewHeadTimer( obj );
    }
    else
    {
        elapsedTime = RtcGetTimerElapsedTime( );
        obj->Timestamp += elapsedTime;

        if( obj->Timestamp < TimerListHead->Timestamp )
        {
            TimerInsertNewHeadTimer( obj );
        }
        else
        {
            TimerInsertTimer( obj );
        }
    }
    CRITICAL_SECTION_END( );
}

static void TimerInsertTimer( TimerEvent_t *obj )
{
    TimerEvent_t* cur = (TimerEvent_t* )TimerListHead;
    TimerEvent_t* next = TimerListHead->Next;

    while( cur->Next != NULL )
    {
        if( obj->Timestamp > next->Timestamp )
        {
            cur = next;
            next = next->Next;
        }
        else
        {
            cur->Next = obj;
            obj->Next = next;
            return;
        }
    }
    cur->Next = obj;
    obj->Next = NULL;
}

static void TimerInsertNewHeadTimer( TimerEvent_t *obj )
{
    TimerEvent_t* cur =(TimerEvent_t* ) TimerListHead;

    if( cur != NULL )
    {
        cur->IsNext2Expire = false;
    }

    obj->Next = cur;
    TimerListHead = obj;
    TimerSetTimeout( (TimerEvent_t* )TimerListHead );
}

bool TimerIsStarted( TimerEvent_t *obj )
{
    return obj->IsStarted;
}
#define TIM_TOLERANCE 1
void TimerIrqHandler( void )
{
    TimerEvent_t* cur;
    TimerEvent_t* next;
    uint32_t old = RtcGetTimerContext( );
    RtcTimerContext_t oldct = RtcTimerContext;
    uint32_t now =  RtcSetTimerContext( );
    RtcTimerContext_t newct = RtcTimerContext;
    uint32_t deltaContext = now - old; // intentional wrap around

    printf("Fact: %i \n", deltaContext);
    printf("old: sec:%i usec:%i\n", (uint32_t)oldct.Time.tv_sec,  (uint32_t)oldct.Time.tv_usec);
    printf("new: sec:%i usec:%i\n", (uint32_t)newct.Time.tv_sec,  (uint32_t)newct.Time.tv_usec);
    // Update timeStamp based upon new Time Reference
    // because delta context should never exceed 2^32


    // Execute immediately the alarm callback
    if ( cur = (TimerEvent_t* )TimerListHead )
    {
    	do{
    		if( cur->Timestamp > deltaContext ){
    			cur->Timestamp -= deltaContext;
    			DBG("TS substracted\n");
    		} else{
    			DBG("TS zeroed\n");
    		    cur->Timestamp = 0;
    		}
    	}while(cur->Next && (cur = cur->Next));

        cur = (TimerEvent_t* )TimerListHead;
        if(cur->Timestamp < TIM_TOLERANCE) {
			TimerListHead = TimerListHead->Next;
			cur->IsStarted = false;
			DBG("TS exec\n");
			ExecuteCallBack( cur->Callback, cur->Context );
        }else {
        	DBG("TS restarted\n");
        	TimerSetTimeout( (TimerEvent_t* )TimerListHead );
        	return;
        }
    }

    // Remove all the expired object from the list
    while( ( TimerListHead != NULL ) && ( TimerListHead->Timestamp < TimerTime(RtcGetTimerElapsedTime()) ) )
    {
        cur = (TimerEvent_t* )TimerListHead;
        TimerListHead = TimerListHead->Next;
        cur->IsStarted = false;
        ExecuteCallBack( cur->Callback, cur->Context );
    }

    // Start the next TimerListHead if it exists AND NOT running
    if( ( TimerListHead != NULL ) && ( TimerListHead->IsNext2Expire == false ) )
    {
        TimerSetTimeout( (TimerEvent_t* )TimerListHead );
    }
}

void TimerStop( TimerEvent_t *obj )
{
    CRITICAL_SECTION_BEGIN( );

    TimerEvent_t* prev = (TimerEvent_t* )TimerListHead;
    TimerEvent_t* cur = (TimerEvent_t* )TimerListHead;

    // List is empty or the obj to stop does not exist
    if( ( TimerListHead == NULL ) || ( obj == NULL ) )
    {
        CRITICAL_SECTION_END( );
        return;
    }

    obj->IsStarted = false;

    if( TimerListHead == obj ) // Stop the Head
    {
        if( TimerListHead->IsNext2Expire == true ) // The head is already running
        {
            TimerListHead->IsNext2Expire = false;
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
                TimerSetTimeout((TimerEvent_t* ) TimerListHead );
            }
            else
            {
                RtcStopAlarm( );
                TimerListHead = NULL;
            }
        }
        else // Stop the head before it is started
        {
            if( TimerListHead->Next != NULL )
            {
                TimerListHead = TimerListHead->Next;
            }
            else
            {
                TimerListHead = NULL;
            }
        }
    }
    else // Stop an object within the list
    {
        while( cur != NULL )
        {
            if( cur == obj )
            {
                if( cur->Next != NULL )
                {
                    cur = cur->Next;
                    prev->Next = cur;
                }
                else
                {
                    cur = NULL;
                    prev->Next = cur;
                }
                break;
            }
            else
            {
                prev = cur;
                cur = cur->Next;
            }
        }
    }
    CRITICAL_SECTION_END( );
}

static bool TimerExists( TimerEvent_t *obj )
{
    TimerEvent_t* cur = (TimerEvent_t* )TimerListHead;

    while( cur != NULL )
    {
        if( cur == obj )
        {
            return true;
        }
        cur = cur->Next;
    }
    return false;
}

void TimerReset( TimerEvent_t *obj )
{
    TimerStop( obj );
    TimerStart( obj );
}

void TimerSetValue( TimerEvent_t *obj, uint32_t value )
{
    uint32_t minValue = 0;
    uint32_t ticks = RtcMs2Tick( value );

    TimerStop( obj );

    minValue = RtcGetMinimumTimeout( );

    if( ticks < minValue )
    {
        ticks = minValue;
    }

    obj->Timestamp = ticks;
    obj->ReloadValue = ticks;
}

TimerTime_t TimerGetCurrentTime( void )
{
    uint32_t now = RtcGetTimerValue( );
    return  RtcTick2Ms( now );
}

TimerTime_t TimerGetElapsedTime( TimerTime_t past )
{
    if ( past == 0 )
    {
        return 0;
    }
    uint32_t nowInTicks = RtcGetTimerValue( );
    uint32_t pastInTicks = RtcMs2Tick( past );

    // Intentional wrap around. Works Ok if tick duration below 1ms
    return RtcTick2Ms( nowInTicks - pastInTicks );
}

static void TimerSetTimeout( TimerEvent_t *obj )
{
	TimerTime elapsed = RtcGetTimerElapsedTime();
    int32_t minTicks= RtcGetMinimumTimeout( );
    obj->IsNext2Expire = true;

    // In case deadline too soon
    if( obj->Timestamp  < ( elapsed + minTicks ) )
    {
    //    obj->Timestamp =elapsed + minTicks;
    }
    RtcSetAlarm( obj->Timestamp );
}

TimerTime_t TimerTempCompensation( TimerTime_t period, float temperature )
{
    return RtcTempCompensation( period, temperature );
}

void TimerProcess( void )
{
    RtcProcess( );
}
