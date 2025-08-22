/*!
 * \file      i2c-board.c
 *
 * \brief     Target board I2C driver implementation
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
#include "stm32wlxx.h"
#include "utilities.h"
#include "board-config.h"
#include "i2c-board.h"

/*!
 *  The value of the maximal timeout for I2C waiting loops
 */
#define TIMEOUT_MAX                                 0x8000

static I2C_HandleTypeDef I2cHandle[3] = { 0 };

static I2cAddrSize I2cInternalAddrSize = I2C_ADDR_SIZE_8;

void I2cMcuInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda )
{
	uint8_t af;
	obj->I2cId = i2cId;
	if(obj->I2cId == I2C_1)  {
		__HAL_RCC_I2C1_CLK_DISABLE( );
		__HAL_RCC_I2C1_CLK_ENABLE( );
		__HAL_RCC_I2C1_FORCE_RESET( );
		__HAL_RCC_I2C1_RELEASE_RESET( );
		 I2cHandle[obj->I2cId].Instance  = ( I2C_TypeDef * )I2C1_BASE;
		 af = GPIO_AF4_I2C1;
	}else if(obj->I2cId == I2C_2) {
		__HAL_RCC_I2C2_CLK_DISABLE( );
	    __HAL_RCC_I2C2_CLK_ENABLE( );
	    __HAL_RCC_I2C2_FORCE_RESET( );
	    __HAL_RCC_I2C2_RELEASE_RESET( );
	    I2cHandle[obj->I2cId].Instance  = ( I2C_TypeDef * )I2C2_BASE;
	    af = GPIO_AF4_I2C2;
	}else if(obj->I2cId == I2C_3) {
		__HAL_RCC_I2C3_CLK_DISABLE( );
	    __HAL_RCC_I2C3_CLK_ENABLE( );
	    __HAL_RCC_I2C3_FORCE_RESET( );
	    __HAL_RCC_I2C3_RELEASE_RESET( );
	    I2cHandle[obj->I2cId].Instance  = ( I2C_TypeDef * )I2C3_BASE;
	    af = GPIO_AF4_I2C3;
	}
    GpioInit( &obj->Scl, scl, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, af );
    GpioInit( &obj->Sda, sda, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, af );
}

void I2cMcuFormat( I2c_t *obj, I2cMode mode, I2cDutyCycle dutyCycle, bool I2cAckEnable, I2cAckAddrMode AckAddrMode, uint32_t I2cFrequency )
{
    I2cHandle[obj->I2cId].Init.Timing = 0x10805D88;
    I2cHandle[obj->I2cId].Init.OwnAddress1 = 0;
    I2cHandle[obj->I2cId].Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle[obj->I2cId].Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle[obj->I2cId].Init.OwnAddress2 = 0;
    I2cHandle[obj->I2cId].Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    I2cHandle[obj->I2cId].Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_I2C_Init( &I2cHandle[obj->I2cId] );
    HAL_I2CEx_ConfigAnalogFilter(&I2cHandle[obj->I2cId], I2C_ANALOGFILTER_ENABLE);
    HAL_I2CEx_ConfigDigitalFilter(&I2cHandle[obj->I2cId], 0);
}

void I2cMcuResetBus( I2c_t *obj )
{
	switch(obj->I2cId){
		case I2C_1: {
			__HAL_RCC_I2C1_FORCE_RESET( );
			__HAL_RCC_I2C1_RELEASE_RESET( );
		}
		break;
		case I2C_2:	{
			__HAL_RCC_I2C2_FORCE_RESET( );
			__HAL_RCC_I2C2_RELEASE_RESET( );
		}
		break;
		case I2C_3: {
			__HAL_RCC_I2C3_FORCE_RESET( );
			__HAL_RCC_I2C3_RELEASE_RESET( );
		}
		break;
		default:{
			assert(0);
		}
	}
    uint8_t af =(obj->I2cId == I2C_1)? GPIO_AF4_I2C1 : \
    		    (obj->I2cId == I2C_2)? GPIO_AF4_I2C2 : GPIO_AF4_I2C3;
    GpioInit( &obj->Scl,  obj->Scl.pin, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, af );
    GpioInit( &obj->Sda,  obj->Sda.pin, PIN_ALTERNATE_FCT, PIN_OPEN_DRAIN, PIN_NO_PULL, af );

    I2cMcuFormat( obj, MODE_I2C, I2C_DUTY_CYCLE_2, true, I2C_ACK_ADD_7_BIT, 0x0090194B );
}

void I2cMcuDeInit( I2c_t *obj )
{

    HAL_I2C_DeInit( & I2cHandle[obj->I2cId] );

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE( );

    GpioInit( &obj->Scl, obj->Scl.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Sda, obj->Sda.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
}

void I2cSetAddrSize( I2c_t *obj, I2cAddrSize addrSize )
{
    I2cInternalAddrSize = addrSize;
}

LmnStatus_t I2cMcuWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    status = ( HAL_I2C_Master_Transmit( & I2cHandle[obj->I2cId], deviceAddr, buffer, size, 2000 ) == HAL_OK ) ? LMN_STATUS_OK : LMN_STATUS_ERROR;

    return status;
}

LmnStatus_t I2cMcuReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    status = ( HAL_I2C_Master_Receive( & I2cHandle[obj->I2cId], deviceAddr, buffer, size, 2000 ) == HAL_OK ) ? LMN_STATUS_OK : LMN_STATUS_ERROR;

    return status;
}

LmnStatus_t I2cMcuWriteMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Write( & I2cHandle[obj->I2cId], deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? LMN_STATUS_OK : LMN_STATUS_ERROR;

    return status;
}

LmnStatus_t I2cMcuReadMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    LmnStatus_t status = LMN_STATUS_ERROR;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Read( & I2cHandle[obj->I2cId], deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? LMN_STATUS_OK : LMN_STATUS_ERROR;

    return status;
}

LmnStatus_t I2cMcuWaitStandbyState( I2c_t *obj, uint8_t deviceAddr )
{
    LmnStatus_t status = LMN_STATUS_ERROR;

    status = ( HAL_I2C_IsDeviceReady( & I2cHandle[obj->I2cId], deviceAddr, 300, 4096 ) == HAL_OK ) ? LMN_STATUS_OK : LMN_STATUS_ERROR;

    return status;
}


void I2C1_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&I2cHandle[I2C_1]);
}
void I2C2_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&I2cHandle[I2C_2]);
}

void I2C3_EV_IRQHandler(void)
{
	HAL_I2C_EV_IRQHandler(&I2cHandle[I2C_3]);
}

void I2C1_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&I2cHandle[I2C_1]);
}

void I2C2_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&I2cHandle[I2C_2]);
}

void I2C3_ER_IRQHandler(void)
{
	HAL_I2C_ER_IRQHandler(&I2cHandle[I2C_3]);
}
