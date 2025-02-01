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

#ifdef __cplusplus

typedef uint8_t NvSum;		// ones-complement checksum

class NvField
{
  Eeprom& eeprom;	// the backing EEPROM device
  typename Eeprom::address addr;		// current read/write address in device
  NvSum sum;		// checksum accumulator

  void note(const void* buf, size_t len)
  {
    const uint8_t* p = static_cast<const uint8_t*>(buf);

    for (size_t i = 0; i < len * sizeof(typename Eeprom::data); i++)
    {
      NvSum t = sum;

      if ((sum += *p++) < t)
        sum += 1;
    }
  }

public:
  NvField(Eeprom& e, typename Eeprom::address a) : eeprom(e), addr(a), sum(0) { }

  // Write buffer to device; update checksum and address.

  bool write(const void* buf, size_t len)
  {
	Eeprom::Result retval = eeprom.write(addr, (Eeprom::data*)buf, len);
	if(retval==Eeprom::Result::OK)
		note(buf, len);
    return retval == Eeprom::Result::OK;;
  }

  // Read from device into buffer; update checksum and address.

  bool read(void* buf, size_t len)
  {
	  Eeprom::Result retval = eeprom.read(addr, (Eeprom::data*)buf, len);
    if(retval == Eeprom::Result::OK)
    	note(buf, len);
    return retval == Eeprom::Result::OK;;
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

  typename Eeprom::address next() const { return addr; }
};

// Nonvolatile storage layout manager for byte-eraseable devices.
class NvStore
{
public:
  Eeprom &eeprom;       // EEPROM driver

  const Eeprom::address base = 1200;          // start address of first record in EEPROM - lora uses about 1160 bytes from 0
  const Eeprom::address end = Eeprom::end;    // end of available EEPROM

  NvStore()
  : eeprom(Eeprom::Instance()){ }
  virtual ~NvStore() = default;

  NvStore(NvStore const&)= delete;
  NvStore& operator= (NvStore const&)= delete;
  static const uint8_t registry[NvVar::NV_LAST_VAR];
  public:
  typedef decltype(eeprom) eeprom_type;

  static NvStore& Instance();
  
  virtual Eeprom::address open(const NvVar& name, void* buf)
  {

    if(name < NvVar::NV_LAST_VAR) {
    		Eeprom::address addr = std::accumulate(registry, &registry[name], base);// size in words
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
      bool res =  payload.write(buf, registry[name]);
      DBG("update %i:%i\n", name, res);
      return res;
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
  NvStore& store;
  mutable TT value;  // RAM image of the nonvolatile variable
  uint16_t  addr;    // EEPROM address of the record payload
  NvVar id;
public:
  const TT min;
  const TT max;
  const TT def;
  NvProperty(const T& min, const T& max, const T& defVal,  const NvVar& id):
	store(NvStore::Instance()),
  	value(std::numeric_limits<T>::max()),
  	addr(store.open(id, &value)),
  	id(id),
  	min(min),
  	max(max),
	def(defVal)
  {
  	  if((value > max) || (value < min)) {
  		  value = def;
  		  if(store.update(id, &def)){
  			  value = def;
  		  }
  	  }
  }

  operator const T& () const {
	if((value > max) || (value < min)) {
		  value = def;
	}
	return value;
  }

  const T& operator = (const T& v)
  {
    value = std::clamp(v, min, max);
    store.update(id, &value);
    return value;
  }

  friend std::ostream& operator<<(std::ostream& os, const NvProperty<T> & nvp)
  {
	  os << "id:" << nvp.id << " addr:" << nvp.addr << " val:" << nvp.value << " min:" << nvp.min << " max: " << nvp.max;
  	  return os << std::endl;
  }
};

#endif
