/* Canbus Event Socket
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

#ifndef CAN_EVENT_SOCKET_H
#define CAN_EVENT_SOCKET_H

#include "drivers/canbus/CanSocket.h"
#include "events/Event.h"
#include "events/EventHandler.h"

/**
 * Event to be sent on CANBUS message reception.
 */
struct CanEvent : Event
{
    uint16_t canTopic; // The topic, aka id of the received message

    CanEvent(uint8_t signal, uint16_t canTopic): 
                                Event{signal}, canTopic(canTopic) {}
};


/**
 * This class is a CanSocket that listens for messages on the CANBUS and
 * posts an event when a message is received.
 * The message can be read using the receive() function of CanSocket.
 */
class CanEventSocket : public CanSocket
{

public:
    /**
     * Create a CanSocket that listens the CANBUS on a certain topic and sends a
     * certain signal to a certain EventHandler when it receives a message.
     * @param handler   who to notify when a message is received
     * @param topic     which topic to listen to on the CANBUS
     * @param sig       the signal of the event sent when a message is received
     */
    CanEventSocket(EventHandler* handler, 
                    const uint16_t topic, 
                    const uint8_t sig):
        CanSocket(topic), handler(handler), signal(sig) {}
    
    /* TODO: Destructor */
    virtual ~CanEventSocket() {};

    /* 
     * Method of the superclass that is executed on every Canbus message reception.
     * The eventSocket overrides adding a postEvent() after every reception.
     */
    virtual void addToMessageList(unsigned char *message, uint8_t size) override
    {
        CanSocket::addToMessageList(message, size);
        handler->postEvent(CanEvent{signal, filter_id});
    }

private:
    EventHandler* handler; // Callback 
    const uint8_t signal;  // Signal of the event to be sent
    
};

#endif /* CAN_EVENT_SOCKET_H */
