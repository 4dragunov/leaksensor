#pragma once

#include <algorithm>
#include <cstring>
#include <numeric>
#include <cassert>
#include <ostream>
#include "eeprom.h"
#include "utilities.h"
#include "LoraMac.h"


#define EEPROM_NV_SIZE  128

typedef enum NvVar {
	FIRST_VAR = 0,
	MODBUS_SLAVE_ID = 0,
	MODBUS_SPEED,
	MODBUS_BITS,
	MODBUS_STOP_BITS,
	MODBUS_PARITY,
	TEMP_CHANNELS,
	DS18B20_RESOLUTION,
	LORA_REGION,
	LORA_TX_DUTYCYCLE,
	LORA_TX_DUTYCYCLE_RND,
	LORA_DEFAULT_DATARATE,
	LORA_ADR_STATE,
	LORA_APP_PORT,
	CHECKSUMM,
	NV_LAST_VAR,
	EEPROM_LENGTH = NV_LAST_VAR
}NvVar;


typedef uint8_t NvSum;		// ones-complement checksum

class NvField
{
  Eeprom& eeprom;	// the backing EEPROM device
  typename Eeprom::address_type addr;		// current read/write address in device
  NvSum sum;		// checksum accumulator

  void note(const void* buf, size_t len)
  {
    const uint8_t* p = static_cast<const uint8_t*>(buf);

    for (size_t i = 0; i < len * sizeof(typename Eeprom::data_type); i++)
    {
      NvSum t = sum;

      if ((sum += *p++) < t)
        sum += 1;
    }
  }

public:
  NvField(Eeprom& e, typename Eeprom::address_type a) : eeprom(e), addr(a), sum(0) { }

  // Write buffer to device; update checksum and address.

  bool write(const void* buf, size_t len)
  {
	bool retval = Eeprom::Instance().write(addr, (Eeprom::data_type*)buf, len);
	if(retval)
		note(buf, len);
    return retval;
  }

  // Read from device into buffer; update checksum and address.

  bool read(void* buf, size_t len)
  {
    bool retval = Eeprom::Instance().read(addr, (Eeprom::data_type*)buf, len);
    if(retval)
    	note(buf, len);
    return retval;
  }

  // Write or read arbitrary types.

  template <typename T>
  bool write(const T& t) { return write(&t, sizeof(t)); }
  template <typename T>
  bool read(T& t) { return read(&t, sizeof(t)); }

  // Write out checksum to device.

  void writeSum() { write(NvSum(~sum)); }

  // Read checksum from device and test it.

  bool testSum() { NvSum s;  return read(s) && NvSum(~sum) == 0; }

  // Get address of field following this one.

  typename Eeprom::address_type next() const { return addr; }
};

#define STORE_BEGIN (0 + sizeof(LoRaMacNvmData_t))
#define STORE_END (STORE_BEGIN + EEPROM_NV_SIZE)

// Nonvolatile storage layout manager for byte-eraseable devices.
class NvStore
{
public:
  Eeprom &eeprom;       // EEPROM driver

  const Eeprom::address_type base = STORE_BEGIN;          // start address of first record in EEPROM
  const Eeprom::address_type end = STORE_END;           // end of available EEPROM

  NvStore()
  : eeprom(Eeprom::Instance()){ }
  virtual ~NvStore() = default;

  NvStore(NvStore const&)= delete;
  NvStore& operator= (NvStore const&)= delete;
  static const uint8_t registry[NvVar::NV_LAST_VAR];
  public:
  typedef decltype(eeprom) eeprom_type;

  static NvStore& Instance();
  
  virtual Eeprom::address_type open(const NvVar& name, void* buf)
  {

    if(name < NvVar::NV_LAST_VAR) {
    		Eeprom::address_type addr = std::accumulate(registry, &registry[name], base);// size in words
    		if(buf) {
				NvField payload(eeprom, addr);
				payload.read(buf, registry[name]);
    		}
    		return addr ;
    } else
    	return end + 1;
  }

  virtual bool update(const NvVar& name, const void* buf)
  {
      NvField payload(eeprom, open(name, nullptr));
      return payload.write(buf, registry[name]);
  }

  void dump() {

	for (int i = 0; i < NvVar::NV_LAST_VAR; i++) {
		union {
		uint8_t bytes[4];
		uint32_t dword;
		}buf = {};
		DBG("id:%i adr: %i, val: %li size: %i\n", i, open((NvVar)i, buf.bytes), buf.dword,  NvStore::registry[i]);
	}
  }
  friend std::ostream& operator<<(std::ostream& os, const NvStore& st);
};
std::ostream& operator<<(std::ostream& os, const NvStore& st);

template<typename T>
using is_class_enum = std::integral_constant<
   bool,
   std::is_enum<T>::value && !std::is_convertible<T, int>::value>;


template < typename T>
class NvProperty
{
  using TT = typename std::conditional<
	    std::is_enum<T>::value,
	    std::underlying_type<T>,
	    std::enable_if<true, T>>::type::type;

  TT value;                   // RAM image of the nonvolatile variable
  uint16_t  addr;        // EEPROM address of the record payload
  NvVar id;
public:
  const TT min;
  const TT max;
  NvProperty(const T& min, const T& max, const T& defVal,  const NvVar& id):
  	value(defVal),
  	addr(NvStore::Instance().open(id, &value)),
  	id(id),
  	min(min),
  	max(max)
  {
  	  DBG("st %p\n", (void*)&NvStore::Instance());
  	  DBG("se %p\n", (void*)&NvStore::Instance().eeprom);
  	  if((value > max) || (value < min)) {
  		  if(NvStore::Instance().update(id, &defVal)){
  			  value = defVal;
  			  DBG("default set\n");
  		  } else {
  			  DBG("default fail \n");
  		  }
  	  } else {
  		  DBG("%i skip\n",addr);
  	  }
  }

  operator const T& () const {
  	return value;
  }

  const T& operator = (const T& v)
  {
    value = std::clamp(v, min, max);
    NvStore::Instance().update(id, &value);
    return value;
  }

  friend std::ostream& operator<<(std::ostream& os, const NvProperty<T> & nvp)
  {
	  os << "id:" << nvp.id << " addr:" << nvp.addr << " val:" << nvp.value << " min:" << nvp.min << " max: " << nvp.max;
  	  return os << std::endl;
  }
};
