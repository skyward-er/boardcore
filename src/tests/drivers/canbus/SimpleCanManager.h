/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once
#include <miosix.h>

#include <cstdlib>
#include <functional>

#include "ActiveObject.h"
#include "drivers/canbus/BusLoadEstimation.h"
#include "drivers/canbus/Canbus.h"
#include "utils/collections/SyncCircularBuffer.h"

using std::function;

using Boardcore::Canbus::Canbus;
using Boardcore::Canbus::CanRXPacket;
using Boardcore::Canbus::CanPacket;
using Boardcore::Canbus::BusLoadEstimation;
using Boardcore::SyncCircularBuffer;
using Boardcore::ActiveObject;

class SimpleCanManager
{
public:
    using RXFunction = function<void(CanRXPacket)>;

    SimpleCanManager(Canbus& can, uint32_t baud_Rate, RXFunction rx_fun)
        : canbus(can), bus_load(baud_Rate), sender(*this), receiver(*this),
          rx_fun(rx_fun)
    {
    }

    void start()
    {
        sender.start();
        receiver.start();
    }

    void stop()
    {
        sender.stop();
        receiver.stop();

        // Empty packet to wakeup sender thread
        tx_packets.put({});
    }

    void send(CanPacket packet)
    {
        if (tx_packets.isFull())
        {
            printf("Pkt drop\n");
        }
        tx_packets.put(packet);
    }

    BusLoadEstimation& getLoadSensor() { return bus_load; }

private:
    class CanSender : public ActiveObject
    {
    public:
        CanSender(SimpleCanManager& parent) : parent(parent) {}
        void run() override
        {
            while (!shouldStop())
            {
                parent.tx_packets.waitUntilNotEmpty();

                if (shouldStop())
                {
                    return;
                }
                CanPacket p = parent.tx_packets.pop();
                parent.canbus.send(p);
                p.timestamp = miosix::getTick();
                parent.bus_load.addPacket(p);
            }
        }

    private:
        SimpleCanManager& parent;
    };

    class CanReceiver : public ActiveObject
    {
    public:
        CanReceiver(SimpleCanManager& parent) : parent(parent) {}
        void run() override
        {
            while (!shouldStop())
            {
                parent.canbus.getRXBuffer().waitUntilNotEmpty();
                while (!parent.canbus.getRXBuffer().isEmpty())
                {
                    CanRXPacket p = parent.canbus.getRXBuffer().pop();
                    parent.bus_load.addPacket(p.packet);
                    parent.rx_fun(p);
                }
            }
        }

    private:
        SimpleCanManager& parent;
    };

    Canbus& canbus;
    BusLoadEstimation bus_load;
    CanSender sender;
    CanReceiver receiver;
    RXFunction rx_fun;
    SyncCircularBuffer<CanPacket, 10> tx_packets;
};