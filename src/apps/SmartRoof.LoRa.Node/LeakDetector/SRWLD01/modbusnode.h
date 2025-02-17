/*
 * modbusnode.h
 *
 *  Created on: Feb 6, 2025
 *      Author: Andrey
 */

#pragma once

#include "messagebus.h"


class ModBusNode : public BusNode {
	friend void StartTaskModBus(void * argument);
public:
	static ModBusNode& Instance(MessageBus &b);
private:
	ModBusNode(MessageBus& mbus);
	virtual ~ModBusNode();
	ModBusNode(ModBusNode const&)= delete;
	ModBusNode& operator= (ModBusNode const&)= delete;

	void onNotify(MessageBus::Message &message);
	void DoTaskModBus();
	osThreadId mModbusTaskHandle;
	MessageBus::Message mSensorData;
	osSemaphoreId mDataChangedSem;
	Gpio_t mDePin;
	std::unique_ptr<ModBus::Slave> mSlave;
};
ModBusNode& InitModBus(MessageBus& mbus);
