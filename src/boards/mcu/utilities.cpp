/*!
 * \file      utilities.h
 *
 * \brief     Helper functions implementation
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
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <string.h>
#include <cstring>

#include "utilities.h"
#include "board-config.h"

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1( void )
{
    return ( ( next = next * 1103515245L + 12345L ) % RAND_LOCAL_MAX );
}

void srand1( uint32_t seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

void memcpy1( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    while( size-- )
    {
        *dst++ = *src++;
    }
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}

int8_t Nibble2HexChar( uint8_t a )
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}

uint32_t Crc32( uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT - 0x04C11DB7
    const uint32_t reversedPolynom = 0xEDB88320;

    // CRC initial value
    uint32_t crc = 0xFFFFFFFF;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint32_t )buffer[i];
        for( uint16_t i = 0; i < 8; i++ )
        {
            crc = ( crc >> 1 ) ^ ( reversedPolynom & ~( ( crc & 0x01 ) - 1 ) );
        }
    }

    return ~crc;
}

uint32_t Crc32Init( void )
{
    return 0xFFFFFFFF;
}

uint32_t Crc32Update( uint32_t crcInit, uint8_t *buffer, uint16_t length )
{
    // The CRC calculation follows CCITT - 0x04C11DB7
    const uint32_t reversedPolynom = 0xEDB88320;

    // CRC initial value
    uint32_t crc = crcInit;

    if( buffer == NULL )
    {
        return 0;
    }

    for( uint16_t i = 0; i < length; ++i )
    {
        crc ^= ( uint32_t )buffer[i];
        for( uint16_t i = 0; i < 8; i++ )
        {
            crc = ( crc >> 1 ) ^ ( reversedPolynom & ~( ( crc & 0x01 ) - 1 ) );
        }
    }
    return crc;
}

uint32_t Crc32Finalize( uint32_t crc )
{
    return ~crc;
}

/*!
  \brief Adds a 8bit number as hex value to a string.
  \param[in,out] dst Start of string buffer, where to append the number string
  \param[in] dstSize The size of the buffer, including the zero byte
  \param[in] num The 8bit number to add
  */
void strcatNum8Hex(char *dst, size_t dstSize, uint8_t num)
{
  char buf[sizeof("FF")]; /* maximum buffer size we need */
  unsigned char hex;

  buf[2] = '\0';
  hex = (char)(num & 0x0F);
  buf[1] = (char)(hex + ((hex <= 9) ? '0' : ('A'-10)));
  hex = (char)((num>>4) & 0x0F);
  buf[0] = (char)(hex + ((hex <= 9) ? '0' : ('A'-10)));
  strncat(dst, const_cast<const char*>(buf), dstSize);
}

void print_buf(const char *title, const unsigned char *buf, size_t buf_len)
{
    size_t i = 0;
    fprintf(stdout, "%s\n", title);
    for(i = 0; i < buf_len; ++i)
    fprintf(stdout, "%02X%s", buf[i],
             ( i + 1 ) % 16 == 0 ? "\r\n" : " " );

}

void print_bytes(std::ostream& out, const char *title, const unsigned char *data, size_t dataLen, bool format) {
    out << title << std::endl;
    out << std::setfill('0');
    for(size_t i = 0; i < dataLen; ++i) {
        out << std::hex << std::setw(2) << (int)data[i];
        if (format) {
            out << (((i + 1) % 16 == 0) ? "\n" : " ");
        }
    }
    out << std::endl;
}
