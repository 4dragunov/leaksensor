#pragma once
#include "messagebus.h"
#include "sensors.h"

class LoraNode : public BusNode {
	friend void StartTaskLoraNode(void * argument);
public:
	static LoraNode& Instance(MessageBus &b);
	osSemaphoreId_t & AppDataChanged() {return mAppDataChangedSem;}
	bool NewDataAvailable;
	void DataSend();
private:
	LoraNode(MessageBus& mbus);
	virtual ~LoraNode();
	LoraNode(LoraNode const&)= delete;
	LoraNode& operator= (LoraNode const&)= delete;

	void onNotify(MessageBus::Message &message);
	void DoTaskLoraNode();

	osThreadId mLoraNodeTaskHandle;
	std::shared_ptr<SummarySensorsData>  mSensorData;
	osSemaphoreId_t mDataChangedSem;
	osSemaphoreId_t mAppDataChangedSem;
	osSemaphoreId_t mAppDataSendSem;
};

LoraNode& InitLoraNode(MessageBus& mbus);
