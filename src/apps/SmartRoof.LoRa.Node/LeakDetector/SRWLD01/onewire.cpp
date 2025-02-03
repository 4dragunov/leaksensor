#include <memory>
#include <cmsis_os.h>

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"
#include "NvmDataMgmt.h"
#include "sensors.h"

extern OneWire::Bus gOWI;
extern OneWire::DS18B20 gDs18b20;

uint8_t ds18b20Sensors = 0;
int16_t ds18b20SensorTemp[8] = {};

osMailQDef(SummarySensors, 1, SummarySensorsData);                    // Define mail queue
osMailQId  gSummarySensorsMq;

void StartTaskOneWire(void const * argument);
osThreadId gOneWireTaskHandle;
osThreadDef(oneWireTask, StartTaskOneWire, osPriorityNormal, 0, 256);

NvProperty<std::underlying_type<OneWire::DS18B20::Resolution>::type> ds18b20_resolution(to_underlying(OneWire::DS18B20::Resolution::SR9BITS),
		to_underlying(OneWire::DS18B20::Resolution::SR12BITS),
		to_underlying(OneWire::DS18B20::Resolution::SR12BITS), NvVar::DS18B20_RESOLUTION);


/* USER CODE BEGIN Header_StartTaskOneWire */
/**
* @brief Function implementing the oneWireTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOneWire */
void StartTaskOneWire(void const * argument)
{
  for(;;)
  {
    if(!ds18b20Sensors) {
    	ds18b20Sensors = gDs18b20.init(static_cast<OneWire::DS18B20::Resolution>((uint8_t)ds18b20_resolution));
    	if(!ds18b20Sensors) {DBG("No sensors!\n");osDelay(200);}
    }else {
    	if(gDs18b20.startMeasure(to_underlying(OneWire::DS18B20::Command::MEASUREALL)) == osOK){

    	if(gDs18b20.waitTempReady(0) == osOK) {
			SummarySensorsData* summaryData = static_cast<SummarySensorsData*>(osMailCAlloc(gSummarySensorsMq, osWaitForever));
			if(summaryData) {
				osEvent ev = osMailGet(DataSampler::Instance(), osWaitForever);
				if(ev.status == osEventMail) {
					DBG("MB MAIL\n");
					summaryData->leakSamples=*static_cast<Samples *>(ev.value.p);
					summaryData->thermal.sensors =  ds18b20Sensors;
					for(uint8_t s = 0; s < ds18b20Sensors; s++)
						OneWire::DS18B20::Error err = gDs18b20.getTempRaw(s, &summaryData->thermal.data[s]);
					gettimeofday(&summaryData->thermal.timestamp, 0);
					gettimeofday(&summaryData->timestamp, 0);
					osMailFree(DataSampler::Instance(), ev.value.p);
					osMailPut(gSummarySensorsMq, (void*)summaryData);  // Send Message
				}
			}
        }else {
        	DBG("temp wait error\n");
        }
	  } else{
		  DBG("failed to start measure\n");
	  }
  }
 }
}


void InitOneWire(void)
{
	gOneWireTaskHandle = osThreadCreate(osThread(oneWireTask), NULL);
	assert(gOneWireTaskHandle);
	gSummarySensorsMq=osMailCreate(osMailQ(SummarySensors), NULL);
	assert(gSummarySensorsMq);
}
