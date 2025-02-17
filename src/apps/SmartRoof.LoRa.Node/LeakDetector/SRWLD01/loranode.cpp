/*
 * Lora.cpp
 *
 *  Created on: Feb 6, 2025
 *      Author: Andrey
 */

#include "loranode.h"
#include "sensors.h"
#include "utilities.h"
#include "cmsis_os.h"
#include "CayenneLpp.h"
#include "LmHandler.h"
#include "board.h"

extern LmHandlerAppData_t AppData;

void StartTaskLoraNode(void * argument){
	static_cast<LoraNode*>(argument)->DoTaskLoraNode();
}

const osThreadAttr_t thread_attr = {
  .name = "LoraNode",
  .stack_size = 512                            // Create the thread stack with a size of 1024 bytes
};

LoraNode::LoraNode(MessageBus &b):
	BusNode(&b),
	NewDataAvailable(false),
	mLoraNodeTaskHandle(osThreadNew(StartTaskLoraNode, this, &thread_attr)),
	mSensorData(nullptr),
	mDataChangedSem(osSemaphoreNew(1, 0, nullptr)),
	mAppDataChangedSem(osSemaphoreNew(1, 0, nullptr)),
	mAppDataSendSem(osSemaphoreNew(1, 0, nullptr)){
	DBG("%s\r\n",__FUNCTION__);
}

LoraNode::~LoraNode() {

}

LoraNode& LoraNode::Instance(MessageBus &b)
{
	static LoraNode n(b);
	return n;
}
void LoraNode::DataSend(){
	osSemaphoreRelease(mAppDataSendSem);
}

void LoraNode::DoTaskLoraNode()
{
	for(;;){
		if(osSemaphoreAcquire(mDataChangedSem, osWaitForever) == osOK) {
			std::shared_ptr<SummarySensorsData> summarySensorsData = static_pointer_cast<SummarySensorsData>(mSensorData);
			uint8_t channel = 0;
		    CayenneLppReset( );
		    DBG("LS MAIL\n");

		    size_t sensors = summarySensorsData->leakSamples.data.ch.wl.size();
		    CayenneLppAddDigitalInput(channel++, sensors );
		    for(size_t i = 0; i < sensors; i++) {
		    	CayenneLppAddRelativeHumidity(channel++, summarySensorsData->leakSamples.data.ch.wl[i] * 100 / 254 );
		    }

		   	CayenneLppAddDigitalInput(channel++, summarySensorsData->thermal.sensors );

		   	for(int i = 0; i < summarySensorsData->thermal.sensors; i++) {
		   		CayenneLppAddTemperature( channel++, summarySensorsData->thermal.data[i] * 100 / 254 );
		   	}
		    //   osMailFree(gTxSensorsMq, summarySensorsData);

		    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );
		   // CayenneLppAddAnalogOutput( channel++, BoardGetModbusId( ) * 100 / 254 );

		    CayenneLppCopy( AppData.Buffer );
		    AppData.BufferSize = CayenneLppGetSize( );
		    DBG("TX size:%i\n",AppData.BufferSize);
		    osSemaphoreRelease(mAppDataChangedSem);
		    summarySensorsData = nullptr;
		    osSemaphoreAcquire(mAppDataSendSem, osWaitForever);
		}
	}
}

void LoraNode::onNotify(MessageBus::Message &message)
{
	mSensorData = message;
	DBG("LoraNode %s\r\n",__FUNCTION__);
	osSemaphoreRelease(mDataChangedSem);
}

LoraNode& InitLoraNode(MessageBus& mbus)
{
	return LoraNode::Instance(mbus);
}
