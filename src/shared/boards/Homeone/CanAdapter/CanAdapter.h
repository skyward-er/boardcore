/* CAN-Bus Event Adapter
 *
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HOMEONE_CAN_ADAPTER_H
#define HOMEONE_CAN_ADAPTER_H

#include <Common.h>
#include <boards/Homeone/Events.h>

#include "drivers/canbus/CanSocket.h"
#include "drivers/canbus/CanManager.h"
#include "drivers/canbus/CanBus.h"
#include "CanInterfaces.h"

namespace HomeoneBoard
{
namespace CanEventAdapter
{

/**
 * This class realizes a Canbus Socket that sends an event to a specific
 * EventHandler whenever a Canbus message is received by the socket.
 * The message can then be read using the receive() function of the superclass.
 */
class CanEventSocket : public CanSocket
{

public:
    CanEventSocket(EventHandler* handler, const uint16_t canTopic) :    
                                            CanSocket(canTopic), handler(handler) {}
    ~CanEventSocket();

    /* Executed on Canbus message reception. */
    void addToMessageList(unsigned char *message, uint8_t size) override
    {
        CanSocket::addToMessageList(message, size);
        handler->postEvent(CanEvent{filter_id});
    }

private:
    EventHandler* handler;
};

/**
 * This class adapts the CanBus driver to the Event paradigm.
 */
class CanAdapter : public Singleton<CanAdapter>
{
    friend class Singleton<CanAdapter>;

public:
    void subscribe(EventHandler* callback, const CanInterfaces::CanTopic topic);
    void post(const Event& ev);
    void post(const StartLaunchEvent& ev);

private:
    CanAdapter();

    const uint8_t BUS_ID = 0;
    CanManager* can_manager;
};

} /* namespace CanEventAdapter */
} /* namespace HomeoneBoard */

#define sCanAdapter HomeoneBoard::CanEventAdapter::CanAdapter::getInstance()

#endif /* HOMEONE_CAN_ADAPTER_H */