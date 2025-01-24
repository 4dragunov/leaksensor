#include "nonvol.h"

template<typename T> const char* TypeName(void);
template<typename T> const char* TypeName(T type) { return TypeName<T>(); }

#define REFLECTION_REGISTER_TYPE(type) \
    template <> const char* TypeName<type>(void) { return #type; }

REFLECTION_REGISTER_TYPE(uint8_t);
REFLECTION_REGISTER_TYPE(uint16_t);
REFLECTION_REGISTER_TYPE(uint32_t);
REFLECTION_REGISTER_TYPE(uint64_t);

const uint8_t NvStore::registry[NV_LAST_VAR] = {
		[MODBUS_SLAVE_ID] = sizeof(uint8_t),    // 0 byte addr
		[MODBUS_SPEED] =  sizeof(uint32_t)/sizeof(Eeprom::data_type),     // 1
		[MODBUS_BITS] = sizeof(uint8_t),        // 3
		[MODBUS_STOP_BITS] = sizeof(uint8_t),   // 4
		[MODBUS_PARITY] = sizeof(uint8_t),      // 5
		[TEMP_CHANNELS] =  sizeof(uint8_t),     // 6
		[DS18B20_RESOLUTION] = sizeof(uint8_t), // 7
		[LORA_REGION] = sizeof(uint8_t),        // 8
		[LORA_TX_DUTYCYCLE] = sizeof(uint32_t)/sizeof(Eeprom::data_type), // 9
		[LORA_TX_DUTYCYCLE_RND] = sizeof(uint16_t)/sizeof(Eeprom::data_type), //11
		[LORA_DEFAULT_DATARATE] = sizeof(uint8_t), //12
		[LORA_ADR_STATE] = sizeof(uint8_t), //13
		[LORA_APP_PORT] = sizeof(uint8_t), //14
		[CHECKSUMM] = sizeof(uint8_t), //15
};

NvStore& NvStore::Instance()
{
      static NvStore s;
      DBG("nvs ss %p\n",(void*)&s);
      DBG("nvs ee %p\n",(void*)&s.eeprom);
      return s;
}

std::ostream& operator<<(std::ostream& os, const NvStore& st)
{
	return os;
}

