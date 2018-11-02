/* Canbus Event Adapter
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

#include "CanEventAdapter.h"

/**
 * Initialize the CanManager, the bus and the sockets
 */
CanEventAdapter::CanEventAdapter()
{
    can_manager = new CanManager(CAN1);

    canbus_init_t can_config = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
    can_manager->addBus<GPIOA_BASE, 11, 12>(can_config);
}


/**
 * Create a new socket that listens to messages on a specific topic
 * @param callback  eventHandler to be notified when a new msg is received
 * @param topic     topic, a.k.a filterId, to be listened on the CANBUS
 * @param signal    signal of the event sent to the callback when a message is received
 * @return          a pointer to the EventSocket that is listening on the given topic
 */
CanEventSocket* CanEventAdapter::subscribe(EventHandler* callback,
                                        const uint16_t topic,
                                        const uint8_t signal)
{
    CanEventSocket* socket = new CanEventSocket(callback, topic, signal);
    socket->open(can_manager->getBus(BUS_ID));

    return socket;
}


/**
 * Send a generic event on the canbus
 * @param ev     Event to be sent
 * @param topic  ID of the message on the CANBUS
 * @return       wether the event was sent successfully or not
 */
bool CanEventAdapter::postEvent(const Event& ev, const uint16_t topic)
{
    const uint8_t* msg = reinterpret_cast<const uint8_t *>(&ev);

    return postMsg(msg, sizeof(Event), topic);
}


/* Send a raw message on the CANBUS. The payload will be cut to 8 bytes if it's longer.
 * @param message  pointer to the messahe to be sent
 * @param len      length on the message (will be trimmed down to 8)        
 * @param topic    ID of the message on the CANBUS
 * @return         wether the message was sent successfully or not
 */
bool CanEventAdapter::postMsg(const uint8_t *message, uint8_t len, const uint16_t topic)
{
    CanBus* bus = can_manager->getBus(BUS_ID);
    len = len > CAN_MAX_PAYLOAD ? CAN_MAX_PAYLOAD : len;

    return bus->send(topic, message, len);
}
