/*!
 * \file      lpm-board.c
 *
 * \brief     Target board low power modes management
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
#include <stdint.h>
#include <stdio.h>
#include "stm32f1xx.h"
#include "utilities.h"
#include "lpm-board.h"

static uint32_t StopModeDisable = 0;
static uint32_t OffModeDisable = 0;

#define STRINGIFY(x) [x] = #x

const char *gLpmModeNames[] = {STRINGIFY(LPM_SLEEP_MODE), STRINGIFY(LPM_STOP_MODE), STRINGIFY(LPM_OFF_MODE) } ;
const char *gLpmModeStateNames [] = {STRINGIFY(LPM_DISABLE), STRINGIFY(LPM_ENABLE)};

void LpmSetOffMode( LpmId_t id, LpmSetMode_t mode )
{
    CRITICAL_SECTION_BEGIN( );

    switch( mode )
    {
        case LPM_DISABLE:
        {
            OffModeDisable |= ( uint32_t )id;
            break;
        }
        case LPM_ENABLE:
        {
            OffModeDisable &= ~( uint32_t )id;
            break;
        }
        default:
        {
            break;
        }
    }
    DBG("OFF mode: %s\n", gLpmModeStateNames[mode]);
    CRITICAL_SECTION_END( );
    return;
}

void LpmSetStopMode( LpmId_t id, LpmSetMode_t mode )
{
    CRITICAL_SECTION_BEGIN( );

    switch( mode )
    {
        case LPM_DISABLE:
        {
            StopModeDisable |= ( uint32_t )id;
            break;
        }
        case LPM_ENABLE:
        {
            StopModeDisable &= ~( uint32_t )id;
            break;
        }
        default:
        {
            break;
        }
    }
    DBG("STOP mode: %s\n", gLpmModeStateNames[mode]);
    CRITICAL_SECTION_END( );
    return;
}

void LpmEnterLowPower( void )
{
    if( StopModeDisable != 0 )
    {
        /*!
        * SLEEP mode is required
        */
    	//DBG("LpmEnter SLEEP\n");
        LpmEnterSleepMode( );
        LpmExitSleepMode( );
    }
    else
    { 
        if( OffModeDisable != 0 )
        {
            /*!
            * STOP mode is required
            */
        	DBG("LpmEnter STOP\n");
            LpmEnterStopMode( );
            LpmExitStopMode( );
        }
        else
        {
            /*!
            * OFF mode is required
            */
        	DBG("LpmEnter OFF\n");
            LpmEnterOffMode( );
            LpmExitOffMode( );
        }
    }
    return;
}

LpmGetMode_t LpmGetMode(void)
{
    LpmGetMode_t mode;

    CRITICAL_SECTION_BEGIN( );

    if( StopModeDisable != 0 )
    {
        mode = LPM_SLEEP_MODE;
    }
    else
    {
        if( OffModeDisable != 0 )
        {
            mode = LPM_STOP_MODE;
        }
        else
        {
            mode = LPM_OFF_MODE;
        }
    }
    DBG("LPM mode: %s\n", gLpmModeStateNames[mode]);
    CRITICAL_SECTION_END( );
    return mode;
}

__weak void LpmEnterSleepMode( void )
{
}

__weak void LpmExitSleepMode( void )
{
}

__weak void LpmEnterStopMode( void )
{
}

__weak void LpmExitStopMode( void )
{
}

__weak void LpmEnterOffMode( void )
{
}

__weak void LpmExitOffMode( void )
{
}
