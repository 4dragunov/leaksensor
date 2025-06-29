#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <cmsis_os.h>

#ifdef __cplusplus

#include <algorithm>

class Eeprom {
public:
	typedef enum {
	  PAGE_0 = 0,
	  PAGE_1 = 1,
	  PAGES_NUM = 2,
	} PageIdx;

	enum class Result{
	  OK = 0,
	  ERROR = 1,
	};

  typedef  uint16_t address;
  typedef  uint16_t data;
  //records bellow is address_type + data_type by size
  typedef  uint32_t data_record; //eeprom records inside page
  typedef  uint32_t page_state_record; //record on the begining of the page about current page status
  typedef enum {
	  CLEAR = 0,
	  ACTIVE,
	  RECEIVING_DATA,
	  UNDEFINED,
	  STATE_COUNT
  } PageState;
  static const address base;       // start address of first record in EEPROM
  static const address end;        // end of available EEPROM
  static const address size;
  static const address pages;
  static  address free;
  static size_t erase_time;
  static size_t read_time;
  static size_t write_time;
  static size_t transfer_time;

  static Eeprom& Instance();
  // Operations on raw buffers.
  virtual Eeprom::Result write(const Eeprom::address addr, const Eeprom::data* buf, size_t len) ;
  virtual Eeprom::Result read(const Eeprom::address addr, Eeprom::data* buf, size_t len) ;
  Eeprom::Result write(const Eeprom::address varId, const Eeprom::data varValue);
  Eeprom::Result read(const Eeprom::address  varId, Eeprom::data *varValue);
  static Eeprom::Result erase();
  bool initialized(){return mInitialized;}
protected:
  PageState ReadPageState(const PageIdx idx);
  static Eeprom::Result SetPageState(const PageIdx idx, const PageState state);
  static Eeprom::Result ClearPage(const PageIdx idx);

  Eeprom::Result GetActivePageIdx(PageIdx *idx);
  Eeprom::Result Init();
  Eeprom::Result RestorePageData(const PageIdx oldPage, const PageIdx newPage);


  Eeprom::Result PageTransfer(const PageIdx activePage, const Eeprom::address varId, const Eeprom::data varValue);

  Eeprom::Result WriteRecord(uint32_t address, const Eeprom::address varId, const Eeprom::data varValue);

private:
  Eeprom();
  virtual ~Eeprom();
  Eeprom(Eeprom const&)= delete;
  Eeprom& operator= (Eeprom const&)= delete;
  static bool mInitialized;
  osMutexId mRwLock;
  static uint32_t pageAddress[PAGES_NUM];
  static PageState pageStates[PAGES_NUM];
  static const page_state_record pageStateValues[PageState::STATE_COUNT -1];
  static const char* pageStateNames[STATE_COUNT];
};
#endif //cpp part

#ifdef __cplusplus
extern "C" {
#endif
bool eepromInit();
bool eepromWrite(const uint16_t addr, const uint8_t* buf, const size_t length);
bool eepromRead(const uint16_t addr, uint8_t* buf, const size_t length);
#ifdef __cplusplus
}
#endif
