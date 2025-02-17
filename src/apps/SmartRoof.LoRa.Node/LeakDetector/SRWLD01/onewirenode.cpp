#include <memory>
#include <cmsis_os.h>

#include "board-config.h"
#include "nonvol.h"
#include "utilities.h"
#include "NvmDataMgmt.h"
#include "sensors-board.h"
#include "onewirenode.h"
#include "sensors.h"

extern OneWire::Bus gOWI;
extern OneWire::DS18B20 gDs18b20;

uint8_t ds18b20Sensors = 0;
int16_t ds18b20SensorTemp[8] = {};

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
  for(;;)
  {
    if(!ds18b20Sensors) {
    	ds18b20Sensors = gDs18b20.init(static_cast<OneWire::DS18B20::Resolution>((uint8_t)ds18b20_resolution));
    	if(!ds18b20Sensors) {DBG("No sensors!\n");osDelay(200);}
    }else {
    	if(gDs18b20.startMeasure(to_underlying(OneWire::DS18B20::Command::MEASUREALL)) == osOK){

    	if(gDs18b20.waitTempReady(0) == osOK) {
    		MessageBus::Message  summaryData = MessageBus::Message(new (osMemoryPoolAlloc(mSummarySamplesMp, osWaitForever)) SummarySensorsData(), [=,this](void* p){
    			DBG("osMemoryPoolFree mSummarySamplesMp %p\r\n", p);
    			osMemoryPoolFree(mSummarySamplesMp, p);
    		});

			if(summaryData) {
				Samples* samples = nullptr;
				if(osMessageQueueGet(DataSampler::Instance().Queue(), &samples, nullptr, osWaitForever) == osOK) {
					DBG("MB MAIL\n");
					std::static_pointer_cast<SummarySensorsData>(summaryData)->leakSamples=*static_cast<Samples *>(samples);
					std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.sensors =  ds18b20Sensors;
					for(uint8_t s = 0; s < ds18b20Sensors; s++)
						OneWire::DS18B20::Error err = gDs18b20.getTempRaw(s, &std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.data[s]);
					gettimeofday(&std::static_pointer_cast<SummarySensorsData>(summaryData)->thermal.timestamp, 0);
					gettimeofday(&std::static_pointer_cast<SummarySensorsData>(summaryData)->timestamp, 0);
					osMemoryPoolFree(DataSampler::Instance().Pool(), samples);
					send(summaryData);
					summaryData = nullptr;
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

void StartTaskOneWire(void * argument){
	static_cast<OneWireNode*>(argument)->DoTaskOneWire();
}

const osThreadAttr_t thread_attr = {
  .name = "OneWireNode",
  .stack_size = 1024                            // Create the thread stack with a size of 1024 bytes
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
