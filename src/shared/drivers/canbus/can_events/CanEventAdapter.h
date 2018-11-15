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

#ifndef CAN_EVENT_ADAPTER_H
#define CAN_EVENT_ADAPTER_H

#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanBus.h>
#include <drivers/canbus/CanUtils.h>

#include "CanEventSocket.h"

/**
 * This class adapts the CanBus driver to Events.
 */
class CanEventAdapter : public Singleton<CanEventAdapter>
{
    friend class Singleton<CanEventAdapter>;

public:
    /*
     * Subscribe an EventHandler to a certain topic, so that it receives an event 
     * whenever a message is received on the CANBUS. 
     */
    CanEventSocket* subscribe(EventHandler* callback, 
                                const uint16_t topic, 
                                const uint8_t signal);

    /* Send a raw message on the CANBUS */
    bool postMsg(const uint8_t *message, const uint8_t len, const uint16_t topic);

private:
    CanEventAdapter();
    ~CanEventAdapter();

    const uint8_t BUS_ID = 0;
    CanManager* can_manager;
};

#define sCanEventAdapter CanEventAdapter::getInstance()

#endif /* CAN_EVENT_ADAPTER_H */
