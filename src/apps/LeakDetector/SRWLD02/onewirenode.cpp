#include <memory>
#include <cmsis_os.h>

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"
#include "NvmDataMgmt.h"
#include "sensors-board.h"
#include "onewire.h"
#include "onewirenode.h"
#include "sensors.h"
#include "ds18b20.h"

extern OneWire::Bus gOWI;
extern OneWire::DS18B20 gDs18b20;

uint8_t ds18b20Sensors = 0;

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

void OneWireNode::DoTaskOneWire(){
  DBG("OneWire Task started\n");

  for(;;)
  {
    	gDs18b20.init(static_cast<OneWire::DS18B20::Resolution>((uint8_t)ds18b20_resolution));
    	if(gDs18b20.sensors()) {
    		if(gDs18b20.startMeasure(to_underlying(OneWire::DS18B20::Command::MEASUREALL)) == osOK){
    			DBG("Temp measurement started!\n");
    		}else{
    			DBG("Failed to start temp measurement on available sensors!\n");
    		}
    	}else{
    		DBG("No onewire sensors available!\n");
    	}

    	MessageBus::Message  summaryData = MessageBus::Message(osMemoryPoolAlloc(mSummarySamplesMp, osWaitForever), [=,this](void* p){
    			DBG("osMemoryPoolFree mSummarySamplesMp %p\r\n", p);
    			osMemoryPoolFree(mSummarySamplesMp, p);
    	});

		if(summaryData) {
				Samples* samples = nullptr;
				if(osMessageQueueGet(DataSampler::Instance().Queue(), &samples, nullptr, osWaitForever) == osOK) {
					DBG("MB MAIL\n");
					std::static_pointer_cast<SummarySensorsData>(summaryData)->leakSamples=*static_cast<Samples *>(samples);
					std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.sensors = 0;
					if(gDs18b20.sensors() && gDs18b20.waitTempReady(0) == osOK) {
						DBG("Reading sensors.\n");
						for(uint8_t sensor = 0; sensor < gDs18b20.sensors(); sensor++) {
							int16_t temp = 0;
							bool read_success = gDs18b20.getTempRaw(sensor, &temp) == OneWire::DS18B20::Error::TEMP_READ;
							DBG("Sensor %i read %s, value %i\n", sensor, read_success? "success" : "failed", temp);
							std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.data[sensor] = read_success? temp : 0xDEAD;
							std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.sensors++;
						}
					}
					if(!std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.sensors)
					{
						DBG("Using CPU thermal sensor data!\n");
						std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.data[0] = samples->data.ch.Ts;
					}
					std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.timestamp = std::chrono::system_clock::now();
					std::static_pointer_cast<SummarySensorsData>(summaryData)->timestamp = std::chrono::system_clock::now();
					osMemoryPoolFree(DataSampler::Instance().Pool(), samples);
					send(summaryData);
					summaryData = nullptr;
					messageDone();
				}
		}else {
			DBG("summaryData is null!\n");
		}
    }
}

void StartTaskOneWire(void * argument){
	static_cast<OneWireNode*>(argument)->DoTaskOneWire();
}

const osThreadAttr_t thread_attr = {
  .name = "OneWireNode",
  .stack_size = 256 * 4,                            // Create the thread stack with a size of 1024 bytes
  .priority = (osPriority_t) osPriorityNormal
};

OneWireNode::OneWireNode():
	MessageBus(),
	BusNode(this),
	mSummarySamplesMp(osMemoryPoolNew(1, sizeof(SummarySensorsData), nullptr)),
	mSummarySamplesMq(osMessageQueueNew(1, sizeof(SummarySensorsData*), nullptr)),
	mOneWireTaskHandle(osThreadNew(StartTaskOneWire, this, &thread_attr)){
	DBG("%s\r\n",__FUNCTION__);
}

OneWireNode::~OneWireNode(){
	osThreadTerminate(mOneWireTaskHandle);
	osMemoryPoolDelete(mSummarySamplesMp);
	osMessageQueueDelete(mSummarySamplesMq);
}

OneWireNode& OneWireNode::Instance(){
	static OneWireNode n;
	return n;
}

void OneWireNode::onNotify(MessageBus::Message &message){
	DBG("OneWireNode %s\r\n",__FUNCTION__);
}


OneWireNode& InitOneWire()
{
	return OneWireNode::Instance();
}
