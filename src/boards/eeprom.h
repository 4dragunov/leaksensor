#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
	extern "C" {
#endif


bool eepromInit();

bool  eepromRead(uint32_t addr, uint8_t *p_data, uint32_t length);
bool  eepromWrite(uint32_t addr, uint8_t *p_data, uint32_t length);

uint8_t  eepromReadByte(uint32_t addr);
bool     eepromWriteByte(uint32_t addr, uint8_t data_in);
uint32_t eepromGetLength(void);
bool     eepromFormat(void);


#ifdef __cplusplus
}
#endif
