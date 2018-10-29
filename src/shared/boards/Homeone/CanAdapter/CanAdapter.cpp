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

#include "CanAdapter.h"

using namespace CanInterfaces;

namespace HomeoneBoard
{
namespace CanEventAdapter
{

/**
 * Initialize the CanManager, the bus and the sockets
 */
CanAdapter::CanAdapter()
{
    can_manager = new CanManager(CAN1);

    canbus_init_t can_config = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
    can_manager->addBus<GPIOA_BASE, 11, 12>(can_config);
}

/**
 * Create a new socket that listens to messages on a specific topic
 */
void CanAdapter::subscribe(EventHandler* callback, const CanTopic topic) 
{
	CanEventSocket* socket = new CanEventSocket(callback, canTopicToInt(topic));
	socket->open(can_manager->getBus(BUS_ID));
}

/**
 * Send a generic event on the canbus
 */
void CanAdapter::post(const Event& ev)
{
    size_t ev_size = canSize(sizeof(Event));
    const uint8_t* msg = reinterpret_cast<const uint8_t*>(&ev);

    CanBus* bus = can_manager->getBus(BUS_ID);
    bus->send(canTopicToInt(CanTopic::CAN_TOPIC_COMMANDS), msg, ev_size);
}

/**
 * Send a startLaunch event on the canbus
 */
void CanAdapter::post(const StartLaunchEvent& ev)
{
    /* Put in the payload only the launch code */
    size_t ev_size = canSize(sizeof(ev.launchCode));
    const uint8_t* msg = reinterpret_cast<const uint8_t*>(&(ev.launchCode));

    CanBus* bus = can_manager->getBus(BUS_ID);
    bus->send(canTopicToInt(CanTopic::CAN_TOPIC_LAUNCH), msg, ev_size);
}

} /* namespace CanEventAdapter */
} /* namespace HomeoneBoard */