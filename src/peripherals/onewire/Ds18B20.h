#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <cmsis_os.h>
#include <sys/time.h>
#include "OneWire.h"

#ifdef __cplusplus

namespace OneWire {


#define MAX_DS18B20_SENSORS 8


	class DS18B20 {

	public:
		enum class Error:uint8_t {
			TEMP_READ,
			ERROR_PARAM,
			TEMP_NOT_READ,
			TEMP_ERROR,
			TEMP_CRC_ERROR
		};

		enum class Resolution: uint8_t {
			SR9BITS  = 0, //93.75ms - 31
			SR10BITS = 1, //187.5ms - 63
			SR11BITS = 2, //375ms - 95
			SR12BITS = 3, //750ms - 127 (31 + 32*i)
		};

		enum class Command:uint8_t {
			CONVERTTEMP 	= 0x44,
			MEASUREALL  	= 0xff
		};

	DS18B20(OneWire::Bus *bus, const  Resolution bits);
	virtual ~DS18B20() = default;
	/**
	 * @brief Initialization of the library
	 * @param *ds18B20: Pointer to @ref Ds18B20_t working Ds18B20 structure
	 * @param *huart: Handle to UART
	 * @param precision: select one from defined precision. It will be set all the same for all found sensors
	 */

	uint8_t init(const  Resolution bits);


	/**
	 * Use this function to start convertion. The temperature will be avalable after the
	 * time needed for the covertion. It depends on the precition set in @ref DS18B20_init
	 * @param *ds18B20: Pointer to @ref Ds18B20_t working Ds18B20 structure
	 * @param sensor: index of the sensor, if specified DS18B20_MEASUREALL the
	 * convertion will be started on all connected sensors.
	 */
	osStatus startMeasure(const uint8_t sensor);

	/**
	 * Check whether the tempereture could be read from the sensor.
	 * @param *ds18B20: Pointer to @ref Ds18B20_t working Ds18B20 structure
	 * @param sensor: index of the sensor. It specified DS18B20_MEASUREALL, the time will be checked on first sensor.
	 * @retval 1 - temperature could be read, 0 - not ready, keep waiting
	 */
	bool isTempReady(const uint8_t sensor);

	osStatus waitTempReady(const uint8_t sensor);

	/**
	 * Retrieve the temperature in the raw value.
	 * @param *ds18B20: Pointer to @ref Ds18B20_t working Ds18B20 structure
	 * @param sensor: index of the sensor
	 * @retval the temperature in steps of 0.0625 degrees centigade.
	 */
	Error  getTempRaw(const uint8_t sensor, int16_t * temp);

	/**
	 * Set the correction of the sensor in raw value. The temperature returned by @ref DS18B20_getTempRaw will be corrected by this value
	 * @param *ds18B20: Pointer to @ref Ds18B20_t working Ds18B20 structure
	 * @param sensor: index of the sensor to apply the correction
	 * @param cor: correction in steps of 0.0625 degrees centigade.
	 */
	void setCorrection(const uint8_t sensor, int16_t cor);

	///Convertion of the raw value to degrees centigare
	inline static double convertToDouble(const int16_t t){
		return (double)t * 0.0625;
	}
	///Convertion of degrees centigare to raw value
	inline static int16_t convertToInt(const double t){
		return (int16_t)((t/0.0625)+0.5);
	}
	private:
		uint8_t mSensorsFound;
		int16_t mCorrection[MAX_DS18B20_SENSORS];
		struct timeval mLastTimeMeasured[MAX_DS18B20_SENSORS];
		uint8_t mROMS[MAX_DS18B20_SENSORS][8];
		uint16_t mTimeNeeded;
		OneWire::Bus *mBus;
	}; //class DS18B20
} //namespace OneWire
#endif
