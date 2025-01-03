#pragma once

#include <algorithm>
#include <cstring>
#include <cassert>
#include <map>
#include "eeprom.h"
#include "utilities.h"

enum class NvVar:uint8_t {
	MODBUS_SLAVE_ID = 1, // 0 not used
	MODBUS_SPEED,
	MODBUS_BITS,
	MODBUS_STOP_BITS,
	MODBUS_PARITY,
	TEMP_CHANNELS,
	DS18B20_RESOLUTION,
	LORA_REGION,
	LORA_TX_DUTYCYCLE_RND,
	LORA_TX_DUTYCYCLE,
	LORA_DEFAULT_DATARATE,
	LORA_ADR_STATE,
	CHECKSUMM,
	COUNT
};


typedef uint8_t NvSum;		// ones-complement checksum

class NvField
{
  Eeprom& eeprom;	// the backing EEPROM device
  typename Eeprom::address_type addr;		// current read/write address in device
  NvSum sum;		// checksum accumulator

  // Update ones-complement checksum with contents of buffer and advance
  // address with its length.

  void note(const void* buf, size_t len)
  {
    const uint8_t* p = static_cast<const uint8_t*>(buf);

    for (size_t i = 0; i < len; i++)
    {
      NvSum t = sum;

      if ((sum += *p++) < t)
        sum += 1;
    }

    addr += len;
  }

public:
  NvField(Eeprom& e, typename Eeprom::address_type a) : eeprom(e), addr(a), sum(0) { }

  // Write buffer to device; update checksum and address.

  void write(const void* buf, size_t len)
  {
    eeprom.write(addr, (Eeprom::data_type*)buf, len);
    note(buf, len);
  }

  // Read from device into buffer; update checksum and address.

  bool read(void* buf, size_t len)
  {
    bool retval = eeprom.read(addr, (Eeprom::data_type*)buf, len);
    note(buf, len);
    return retval;
  }

  // Write or read arbitrary types.

  template <typename T>
  void write(const T& t) { write(&t, sizeof(t)); }
  template <typename T>
  bool read(T& t) { return read(&t, sizeof(t)); }

  // Write out checksum to device.

  void writeSum() { write(NvSum(~sum)); }

  // Read checksum from device and test it.

  bool testSum() { NvSum s;  return read(s) && NvSum(~sum) == 0; }

  // Get address of field following this one.

  typename Eeprom::address_type next() const { return addr; }
};

#define STORE_BEGIN 0
#define STORE_END 128



// Nonvolatile storage layout manager for byte-eraseable devices.
class NvStore
{
  Eeprom& eeprom;       // EEPROM driver

  const typename std::remove_reference<decltype(eeprom)>::type::address_type base = STORE_BEGIN;          // start address of first record in EEPROM
  const typename std::remove_reference<decltype(eeprom)>::type::address_type end = STORE_END;           // end of available EEPROM

  const size_t maxNameLen = 16;  // maximum record name length
  NvStore()
  : eeprom(Eeprom::Instance()){ }
  virtual ~NvStore() = default;

  NvStore(NvStore const&)= delete;
  NvStore& operator= (NvStore const&)= delete;
  const std::map<NvVar, uint8_t> registry;
  public:
  typedef decltype(eeprom) eeprom_type;

  static NvStore& Instance()
  {
        static NvStore s;
        return s;
  }
  
  typename std::remove_reference<decltype(eeprom)>::type::address_type open(const NvVar& name, void* buf, size_t len)
  {
    typename std::remove_reference<decltype(eeprom)>::type::address_type addr = base;
    //check than each address has unique data length associated
   // assert((registry::find(name) != NvStore::registry.end() && registry[name] == len));

    for(const auto& [key, value] : registry) {
    	if(key == name){
    		addr += to_underlying(name);
    		NvField payload(eeprom, addr);
    		payload.read(buf, len);
    		return addr ;
    	}else
    		addr += value;
    }
    return 0;
  }

  void update(typename std::remove_reference<decltype(eeprom)>::type::address_type  addr, const void* buf, size_t len)
  {
    if (addr)
    {
      NvField payload(eeprom, addr);
      payload.write(buf, len);
    }
  }

};


// Nonvolatile variable.

template < typename T>
class NvProperty
{
 T mMin;
 T mMax;
 T mV;                  // RAM image of the nonvolatile variable
public:
  NvStore& store;       // EEPROM storage layout manager
  uint16_t  addr;          // EEPROM address of the record payload

  NvProperty(const T& min, const T& max, const T& _t,  const NvVar& id)
  : mMin(min), mMax(max), mV(_t), store(NvStore::Instance()), addr(store.open(id, &mV, sizeof(mV)))
  {
	 // std::clamp(mV, min, max);
  }

  // Read variable from RAM image.
  
  operator const T& () const {
	  return mV;
  }

  // Write variable to RAM and through to backing store.
  
  const T& operator = (const T& v)
  { 

    mV = v;//std::clamp(v, mMin, mMax);
    store.update(addr, &mV, sizeof(mV));
    return mV;
  }
};  




