#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus

#define _C extern "C"

#define EEPROM_START 0
#define EEPROM_SIZE 256


template<typename A, typename D>
class IEeprom {
public:
	 typedef  A address_type;
	 typedef  D data_type;
	 virtual bool write(address_type addr, const data_type* buf, size_t len) = 0;
	 virtual bool read(address_type addr, data_type* buf, size_t len) = 0;
};

class Eeprom:public IEeprom<uint16_t, uint16_t>
{
public:
  static Eeprom& Instance()
  {
      static Eeprom s;
      return s;
  }
  const address_type base = EEPROM_START;          // start address of first record in EEPROM
  const address_type end = EEPROM_SIZE;           // end of available EEPROM
  const address_type size = end - base;
  virtual bool erase() { return false; }
  // Operations on raw buffers.
  
  virtual bool write(address_type addr, const data_type* buf, size_t len);
  virtual bool read(address_type addr, data_type* buf, size_t len);

  // Operations on known data types.
  
  template <typename T>
  bool write(address_type addr, const T& t) { return write(base + addr, (data_type*) &t, sizeof(t)/sizeof(data_type)); }

  template <typename T>
  bool read(address_type addr, T& t) { return read(base + addr, (data_type*) &t, sizeof(t)/sizeof(data_type)); }
private:
  Eeprom();
  ~Eeprom();
  Eeprom(Eeprom const&)= delete;
  Eeprom& operator= (Eeprom const&)= delete;
  bool mInitialized;
};

#else

#define _C

#endif

_C bool eepromInit();
_C bool eepromWrite(uint16_t addr, const void* buf, size_t length);
_C bool eepromRead(uint16_t addr, const void* buf, size_t length);




