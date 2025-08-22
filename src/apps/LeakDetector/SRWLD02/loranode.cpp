/*
 * Lora.cpp
 *
 *  Created on: Feb 6, 2025
 *      Author: Andrey
 */

#include "loranode.h"

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
  .stack_size = 128 * 4, //512
  .priority = (osPriority_t) osPriorityNormal
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
	DBG("Lora node task started");
	for(;;){
		if(osSemaphoreAcquire(mDataChangedSem, osWaitForever) == osOK) {
			uint8_t channel = 0;
		    CayenneLppReset( );
		    DBG("LS MAIL\n");

		    size_t sensors = mSensorData->leakSamples.data.ch.wl1.size() + mSensorData->leakSamples.data.ch.wl2.size();
		    CayenneLppAddDigitalInput(channel++, sensors );
		    for(size_t i = 0; i < sensors; i++) {
		    	CayenneLppAddRelativeHumidity(channel++, (i < mSensorData->leakSamples.data.ch.wl1.size())? mSensorData->leakSamples.data.ch.wl1[i] :
		    																				   mSensorData->leakSamples.data.ch.wl2[i - mSensorData->leakSamples.data.ch.wl1.size()]);
		    }

		   	CayenneLppAddDigitalInput(channel++, mSensorData->thermal.sensors );
		   	if(mSensorData->thermal.sensors) {
				for(int i = 0; i < mSensorData->thermal.sensors; i++) {
					CayenneLppAddTemperature( channel++, mSensorData->thermal.data[i] / 10.0 );
				}
		   	}else{
		   		//cpu thermal data allways present at index 0 if ds18b20 sensors not available
		   		CayenneLppAddTemperature( channel++, mSensorData->thermal.data[0] / 10.0 );
		   	}
		    CayenneLppAddAnalogInput( channel++, BoardGetBatteryLevel( ) * 100 / 254 );
		   // CayenneLppAddAnalogOutput( channel++, BoardGetModbusId( ) * 100 / 254 );

		    CayenneLppCopy( AppData.Buffer );
		    AppData.BufferSize = CayenneLppGetSize( );
		    DBG("TX size:%i\n",AppData.BufferSize);
		    osSemaphoreRelease(mAppDataChangedSem);
		    mSensorData = nullptr;
		    messageDone();
		    osSemaphoreAcquire(mAppDataSendSem, osWaitForever);
		}
	}
}

void LoraNode::onNotify(MessageBus::Message &message)
{
	mSensorData = static_pointer_cast<SummarySensorsData>(message);
	DBG("LoraNode %s\r\n",__FUNCTION__);
	osSemaphoreRelease(mDataChangedSem);
}

LoraNode& InitLoraNode(MessageBus& mbus)
{
	return LoraNode::Instance(mbus);
}
