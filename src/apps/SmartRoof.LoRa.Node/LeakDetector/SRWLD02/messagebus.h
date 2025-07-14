/*
 * messagebus.h
 *
 *  Created on: Feb 5, 2025
 *      Author: Andrey
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <memory>
#include <functional>
#include <iostream>
#include "cmsis_os.h"

class MessageBus
{
public:
	typedef typename std::shared_ptr<void> Message;
    MessageBus();
    virtual ~MessageBus();

    void addReceiver(std::function<void(Message&)> messageReceiver);

    void sendMessage(Message &message);
private:
    std::vector<std::function<void(Message&)>> receivers;
};

class BusNode
{
public:
    BusNode(MessageBus* messageBus);
    virtual ~BusNode();
protected:
    MessageBus* messageBus;
    MessageBus::Message message;
    std::function<void(MessageBus::Message&)> getNotifyFunc();

    void send(MessageBus::Message &message);

    virtual void onNotify(MessageBus::Message &message);
    virtual void messageDone();

};
