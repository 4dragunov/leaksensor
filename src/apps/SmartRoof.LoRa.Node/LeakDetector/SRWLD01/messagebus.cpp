/*
 * messagebus.cpp
 *
 *  Created on: Feb 5, 2025
 *      Author: Andrey
 */
#include <cassert>
#include "messagebus.h"
#include "cmsis_os2.h"

MessageBus::MessageBus():
		receivers(){
}

MessageBus::~MessageBus() {
}

void MessageBus::addReceiver(std::function<void(Message&)> messageReceiver)
{
	receivers.push_back(messageReceiver);
}

void MessageBus::sendMessage(Message &message)
{
	for (auto reciever: receivers) {
		reciever(message);
	}
}

BusNode::BusNode(MessageBus* messageBus):messageBus(messageBus)
{
	this->messageBus->addReceiver(getNotifyFunc());
}

std::function<void(MessageBus::Message&)> BusNode::getNotifyFunc()
{
	auto messageListener = [=, this](MessageBus::Message &message) -> void {
	    this->onNotify(message);
	};
	return messageListener;
}

void BusNode::send(MessageBus::Message &message)
{
	 messageBus->sendMessage(message);
}

void BusNode::onNotify(MessageBus::Message &message)
{
	assert(0);
}
