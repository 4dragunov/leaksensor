#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os.h>

#ifdef __cplusplus
#include <algorithm>
#define __C extern "C"

#define FLASH_SIZE_KB             (uint32_t)(*((uint32_t *)FLASHSIZE_BASE)&0xFFFF)
#define FLASH_SIZE                (uint32_t)(FLASH_SIZE_KB * 1024U)

#define EEPROM_START 0
#define EEPROM_SIZE (FLASH_PAGE_SIZE/sizeof(uint16_t))


class Eeprom {
	friend uint16_t eepromReadWord(uint32_t addr);
	friend bool eepromWriteWord(uint32_t addr, uint16_t data_in);
	friend uint32_t eepromGetLength(void);
public:
  typedef  uint16_t address_type;
  typedef  uint16_t data_type;
  static Eeprom& Instance();
  static const address_type base;       // start address of first record in EEPROM
  static const address_type end;        // end of available EEPROM
  const address_type size = end - base;
  virtual bool erase() { return false; }
  // Operations on raw buffers.
  //len in data_type
  virtual bool write(address_type addr, const data_type* buf, size_t len) ;
  virtual bool read(address_type addr, data_type* buf, size_t len) ;

  // Operations on known data types.
  
  template <typename T>
  bool write(address_type addr, const T& t) {
	return write(base + addr, (data_type*) &t, std::max(1u, sizeof(t)/sizeof(data_type)));
  }

  template <typename T>
  bool read(address_type addr, T& t) {
	return read(base + addr, (data_type*) &t, std::max(1u, sizeof(t)/sizeof(data_type)));
  }
  bool WriteWord(uint32_t addr, uint16_t data_in);
  uint16_t ReadWord(uint32_t addr);
protected:
  uint16_t PageTransfer(uint16_t VirtAddress, uint16_t Data);
  uint16_t VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
  uint16_t FindValidPage(uint8_t Operation);
  uint16_t Format(void);
  uint16_t WriteVariable(uint16_t VirtAddress, uint16_t Data);
  uint16_t ReadVariable(uint16_t VirtAddress, uint16_t* Data);
  bool VerifyPageFullyErased(uint32_t Address);
  uint16_t Init(void);

  uint32_t GetLength(void);
private:
  Eeprom();
  virtual ~Eeprom();
  Eeprom(Eeprom const&)= delete;
  Eeprom& operator= (Eeprom const&)= delete;
  static bool mInitialized;
  osMutexId mRwLock;
  /* Global variable used to store variable value in read sequence */
  uint16_t DataVar;
};
uint16_t eepromReadWord(uint32_t addr);
bool eepromWriteWord(uint32_t addr, uint16_t data_in);
#else
	#define __C
#endif

__C bool eepromInit();
__C bool eepromWrite(uint16_t addr, const void* buf, size_t length);
__C bool eepromRead(uint16_t addr, const void* buf, size_t length);

