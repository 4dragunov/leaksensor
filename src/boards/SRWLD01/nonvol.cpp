#include "nonvol.h"




const std::map<NvVar, uint8_t> NvStorage::registry = {
		{MODBUS_SLAVE_ID,sizeof(uint8_t)},
		{MODBUS_SPEED, sizeof(uint32_t)},
		{MODBUS_BITS,sizeof(uint8_t)},
		{MODBUS_STOP_BITS,sizeof(uint8_t)},
		{MODBUS_PARITY,sizeof(uint8_t)},
		{TEMP_CHANNELS, sizeof(uint8_t)},
		{DS18B20_RESOLUTION,sizeof(uint8_t)},
		{CHECKSUMM,sizeof(uint8_t)}
};
