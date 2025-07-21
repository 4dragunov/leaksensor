/*
 * modbusnode.h
 *
 *  Created on: Feb 6, 2025
 *      Author: Andrey
 */

#pragma once

#include "messagebus.h"

class OneWireNode : public MessageBus, BusNode {
	friend void StartTaskOneWire(void * argument);
	friend OneWireNode& InitOneWire();
public:
	static OneWireNode& Instance();
	//operator MessageBus&(){ return *this;}
//	operator osThreadId&() {return mOneWireTaskHandle;}
	//operator osMailQId&() {return mSummarySensorsMq;}

private:
	OneWireNode();
	virtual ~OneWireNode();
	OneWireNode(OneWireNode const&)= delete;
	OneWireNode& operator= (OneWireNode const&)= delete;
	void onNotify(MessageBus::Message &message);
	void DoTaskOneWire();
	osMemoryPoolId_t mSummarySamplesMp;
	osMessageQueueId_t mSummarySamplesMq;
	osThreadId_t mOneWireTaskHandle;
};

OneWireNode& InitOneWire();
