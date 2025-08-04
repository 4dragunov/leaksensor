/*!
 * \file      i2c.h
 *
 * \brief     I2C driver implementation
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
#ifndef __I2C_H__
#define __I2C_H__

#include "utilities.h"
#include "gpio.h"
#include <cmsis_os.h>

/*!
 * I2C peripheral ID
 */
typedef enum
{
    I2C_1,
    I2C_2,
	I2C_3
}I2cId_t;

/*!
 * I2C object type definition
 */
typedef struct
{
    I2cId_t I2cId;
    Gpio_t Scl;
    Gpio_t Sda;
}I2c_t;

#ifdef __cplusplus
extern "C"
{
#endif

/*!
 * \brief Initializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 * \param [IN] scl  I2C Scl pin name to be used
 * \param [IN] sda  I2C Sda pin name to be used
 */
void I2cInit( I2c_t *obj, I2cId_t i2cId, PinNames scl, PinNames sda );

/*!
 * \brief DeInitializes the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cDeInit( I2c_t *obj );

/*!
 * \brief Reset the I2C object and MCU peripheral
 *
 * \param [IN] obj  I2C object
 */
void I2cResetBus( I2c_t *obj );

/*!
 * \brief Write data to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] data             data to write
 */
LmnStatus_t I2cWrite( I2c_t *obj, uint8_t deviceAddr, uint8_t data );

/*!
 * \brief Write data buffer to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of bytes to write
 */
LmnStatus_t I2cWriteBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Write data at addr to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] data             data to write
 */
LmnStatus_t I2cWriteMem( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t data );

/*!
 * \brief Write data buffer starting at addr to the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [IN] buffer           data buffer to write
 * \param [IN] size             number of bytes to write
 */
LmnStatus_t I2cWriteMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Read data from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [OUT] data            data to read
 */
LmnStatus_t I2cRead( I2c_t *obj, uint8_t deviceAddr, uint8_t *data );

/*!
 * \brief Read data buffer from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [OUT] buffer          data buffer to read
 * \param [IN] size             number of data bytes to read
 */
LmnStatus_t I2cReadBuffer( I2c_t *obj, uint8_t deviceAddr, uint8_t *buffer, uint16_t size );

/*!
 * \brief Read data at addr from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] data            data to read
 */
LmnStatus_t I2cReadMem( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *data );

/*!
 * \brief Read data buffer starting at addr from the I2C device
 *
 * \param [IN] obj              I2C object
 * \param [IN] deviceAddr       device address
 * \param [IN] addr             data address
 * \param [OUT] buffer          data buffer to read
 * \param [IN] size             number of data bytes to read
 */
LmnStatus_t I2cReadMemBuffer( I2c_t *obj, uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );

LmnStatus_t I2cWaitStandbyState( I2c_t *obj, uint8_t deviceAddr );

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus

class I2C: I2c_t {
	osMutexId_t mMtx;
	PinNames mSclPin;
	PinNames mSdaPin;
public:
	I2C(I2cId_t i2cId, PinNames scl, PinNames sda):I2c_t(), mMtx(osMutexNew(NULL)),
	mSclPin(scl), mSdaPin(sda){
		this->I2cId = i2cId;
	};
	virtual ~I2C() {I2cDeInit(this);};
	virtual void init(){I2cInit( this, this->I2cId, mSclPin, mSdaPin );};
	virtual osStatus_t lock(uint32_t ms=osWaitForever) {return  osMutexAcquire(mMtx, ms);};
	virtual osStatus_t unlock() {return osMutexRelease(mMtx);};
	virtual void reset() {I2cResetBus(this);};
	virtual void frequency(uint32_t freq) {};
	virtual LmnStatus_t write(uint8_t deviceAddr, uint8_t data, bool cmd = false) {return I2cWrite( this , deviceAddr, data );};
	virtual LmnStatus_t write(uint8_t deviceAddr, uint8_t *buffer, uint16_t size, bool cmd = false) {return I2cWriteBuffer(this, deviceAddr, buffer, size);};
	virtual LmnStatus_t writeMem(uint8_t deviceAddr, uint16_t addr, uint8_t data ) {return I2cWriteMem( this, deviceAddr, addr, data );};
	virtual LmnStatus_t writeMem(uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size ) {return I2cWriteMemBuffer(this, deviceAddr, addr, buffer, size);};

	virtual LmnStatus_t read(uint8_t deviceAddr, uint8_t *data ) {return I2cRead(this, deviceAddr, data);};
	virtual LmnStatus_t read( uint8_t deviceAddr, uint8_t *buffer, uint16_t size ) {return I2cReadBuffer(this, deviceAddr, buffer, size);};
	virtual LmnStatus_t readMem( uint8_t deviceAddr, uint16_t addr, uint8_t *data ) {return I2cReadMem(this, deviceAddr, addr, data);};
	virtual LmnStatus_t readMem( uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size ){return I2cReadMemBuffer(this, deviceAddr, addr, buffer, size);};
};
#endif
#endif // __I2C_H__
