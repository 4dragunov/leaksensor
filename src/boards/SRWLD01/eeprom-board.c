/*!
 * \file      eeprom-board.c
 *
 * \brief     Target board EEPROM driver implementation
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
#include "stm32f1xx.h"
#include "utilities.h"
#include "eeprom-board.h"
#include "eeprom.h"

LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    assert_param( ( FLASH_EEPROM_BASE + addr ) >= FLASH_EEPROM_BASE );
    assert_param( buffer != NULL );
    assert_param( size < ( FLASH_EEPROM_END - FLASH_EEPROM_BASE ) );

    CRITICAL_SECTION_BEGIN( );
    status =  eepromWrite(addr, buffer, size)? LMN_STATUS_OK : LMN_STATUS_ERROR;
    CRITICAL_SECTION_END( );

    return status;
}

LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    assert_param( ( FLASH_EEPROM_BASE + addr ) >= FLASH_EEPROM_BASE );
    assert_param( buffer != NULL );
    assert_param( size < ( FLASH_EEPROM_END - FLASH_EEPROM_BASE ) );
    return eepromRead(addr, buffer, size)? LMN_STATUS_OK : LMN_STATUS_ERROR;
}

void EepromMcuSetDeviceAddr( uint8_t addr )
{
    assert_param( LMN_STATUS_ERROR );
}

LmnStatus_t EepromMcuGetDeviceAddr( void )
{
    assert_param( LMN_STATUS_ERROR );
    return 0;
}
