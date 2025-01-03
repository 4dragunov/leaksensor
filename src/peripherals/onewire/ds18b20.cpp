#include <algorithm>
#include "ds18b20.h"
#include "OneWire.h"
#include "cmsis_os.h"
#include "utilities.h"

namespace OneWire {

#define BITS2CMD(x) (31 + (32 * x))
#define usec2sec(x)  (x / (1000 * 1000))
#define sec2usec(x)  (x * 1000 * 1000)
#define msecOnSec (1000 * 1000)

const uint16_t gResolution2Ms[] = {94, 186, 375, 750};

struct timeval operator-(const struct timeval& a, const struct timeval& b)
{
	struct timeval result;
	uint64_t usec = sec2usec(a.tv_sec) - sec2usec(b.tv_sec) + (a.tv_usec - b.tv_usec);
	result.tv_sec = usec2sec(usec);
	result.tv_usec = usec % msecOnSec;
	return result;
}

bool operator>=(const struct timeval& a, const uint64_t& b)
{
	uint64_t msec = (sec2usec(a.tv_sec) + a.tv_usec) / 1000;
	return msec >= b;
}

uint64_t time2usec(const struct timeval& a)
{
	return sec2usec(a.tv_sec) + a.tv_usec ;
}

uint64_t time2msec(const struct timeval& a)
{
	return time2usec(a) / 1000;
}

DS18B20::DS18B20(OneWire::Bus *bus, const  Resolution bits):
	mSensorsFound(0),
	mCorrection{},
	mLastTimeMeasured{},
	mROMS(),
	mTimeNeeded(gResolution2Ms[to_underlying(bits)]),
	mBus(bus)
{
}

uint8_t DS18B20::init(const  Resolution bits)
{   
    uint8_t status =  mBus->first();

    //Looking for the sensors while there are avalable and their amount could be stored
    while (status && mSensorsFound < MAX_DS18B20_SENSORS) {
        //Save all ROMs
        mBus->getFullROM(mROMS[mSensorsFound]);
        //Check the CRC
        uint8_t crc = mBus->crc8(&mROMS[mSensorsFound][0], 7);
        if ( crc == mROMS[mSensorsFound][7]) {
           mSensorsFound++;
        }
        //Looking for the next
        status =  mBus->next();
    }

    uint8_t data[] = {  to_underlying(Bus::Command::SKIPROM),
    					to_underlying(Bus::Command::WSCRATCHPAD),
                        0x7F, //0b0111 1111 //temp high
                        0xFF, //0b1111 1111 //temp low 
						BITS2CMD(to_underlying(bits)) };

    if ( mBus->reset()) {
        //Select all sensors.
    	 mBus->send(data, sizeof(data));
    	 mTimeNeeded = gResolution2Ms[to_underlying(bits)];
    }
    return mSensorsFound;
}

osStatus DS18B20::startMeasure(const uint8_t sensor)
{
    if ( sensor < mSensorsFound || sensor == to_underlying(DS18B20::Command::MEASUREALL) ) {

        struct timeval now;
        if(!gettimeofday(&now, nullptr)) {
			if (sensor == to_underlying(Command::MEASUREALL)) {
				if (mBus->reset()) {
					//Select all sensors. It's faster
					mBus->send(to_underlying(Bus::Command::SKIPROM));
					mBus->send(to_underlying(Command::CONVERTTEMP));
					for(uint8_t i=0; i< mSensorsFound; i++) {
					   mLastTimeMeasured[i] = now;
					}
					return osOK;
				}
			} else {
				if (mBus->reset()) {
					mBus->select(mROMS[sensor]);
					mBus->send(to_underlying(Command::CONVERTTEMP));
					mLastTimeMeasured[sensor] = now;
					return osOK;
				}
			}
        }
    }
    return osErrorResource;
}

bool DS18B20::isTempReady(const uint8_t sensor)
{
	struct timeval now;
	gettimeofday(&now, nullptr);
    if ((sensor != to_underlying(Command::MEASUREALL)) && (sensor >= mSensorsFound)) return false;

    return ((now - mLastTimeMeasured[(sensor == to_underlying(Command::MEASUREALL))? 0 : sensor]) >= mTimeNeeded);
}

osStatus DS18B20::waitTempReady(const uint8_t sensor)
{
	struct timeval now;
	gettimeofday(&now, nullptr);

    if (sensor != to_underlying(Command::MEASUREALL) && (sensor >= mSensorsFound)) return osErrorParameter;
    uint32_t delay = std::min(time2msec(now - mLastTimeMeasured[(sensor == to_underlying(Command::MEASUREALL))? 0 : sensor]), (uint64_t) mTimeNeeded);
    return osDelay(delay);
}

DS18B20::Error DS18B20::getTempRaw(const uint8_t sensor, int16_t * temp)
{
    if (sensor != to_underlying(Command::MEASUREALL) && sensor >= mSensorsFound)
        return Error::TEMP_NOT_READ;
    
    if(!temp)
    	return Error::ERROR_PARAM;

    if (!mBus->reset()){
        return Error::TEMP_NOT_READ;
    }
    
    if (sensor == to_underlying(Command::MEASUREALL)) {
        mBus->send(to_underlying(Bus::Command::SKIPROM));
    } else {
        mBus->select(mROMS[sensor]);
    }
    mBus->send(to_underlying(Bus::Command::RSCRATCHPAD));
    
    uint8_t data[12];
    uint16_t s = 0;
    
    for (uint8_t j = 0; j < 9; j++) {           // we need 9 bytes
        data[j] = mBus->receive();
        s += data[j];
    }
    //The CRC algorithm has an error. If all bytes are zeros the CRC will be ok
    //So this check is agains it
    if (s==0) {
        return Error::TEMP_CRC_ERROR;
    }

    if (mBus->crc8(data, 8) != data[8]) {
        return Error::TEMP_ERROR;
    }
    
    //temp calculation
    int16_t raw = (data[1] << 8) | data[0];
    uint8_t cfg = (data[4] & 0x60);

    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
    *temp = raw + mCorrection[(sensor == to_underlying(Command::MEASUREALL))? 0 : sensor];
    return Error::TEMP_READ;
}

void DS18B20::setCorrection(const uint8_t sensor, const int16_t cor)
{
    if (sensor >= mSensorsFound)
    	return;
    else
    	mCorrection[sensor] = cor;
}

}//namespace OneWire
