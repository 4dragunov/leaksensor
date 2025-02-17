#pragma once
#include "messagebus.h"

class LoraNode : public BusNode {
	friend void StartTaskLoraNode(void * argument);
public:
	static LoraNode& Instance(MessageBus &b);
private:
	LoraNode(MessageBus& mbus);
	virtual ~LoraNode();
	LoraNode(LoraNode const&)= delete;
	LoraNode& operator= (LoraNode const&)= delete;

	void onNotify(MessageBus::Message &message);
	void DoTaskLoraNode();

	osThreadId mLoraNodeTaskHandle;
	MessageBus::Message mSensorData;
	osSemaphoreId mDataChangedSem;
};

LoraNode& InitLoraNode(MessageBus& mbus);
