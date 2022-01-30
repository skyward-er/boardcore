/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

class SimpleCanManager
{
public:
    using RXFunction = function<void(Boardcore::Canbus::CanRXPacket)>;

    SimpleCanManager(Boardcore::Canbus::CanbusDriver& can, uint32_t baud_Rate,
                     RXFunction rxFun)
        : canbus(can), busLoad(baud_Rate), sender(*this), receiver(*this),
          rxFun(rxFun)
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
        txPackets.put({});
    }

    void send(Boardcore::Canbus::CanPacket packet)
    {
        if (txPackets.isFull())
        {
            printf("Pkt drop\n");
        }
        txPackets.put(packet);
    }

    Boardcore::Canbus::BusLoadEstimation& getLoadSensor() { return busLoad; }

private:
    class CanSender : public Boardcore::ActiveObject
    {
    public:
        explicit CanSender(SimpleCanManager& parent) : parent(parent) {}
        void run() override
        {
            while (!shouldStop())
            {
                parent.txPackets.waitUntilNotEmpty();

                if (shouldStop())
                {
                    return;
                }
                Boardcore::Canbus::CanPacket p = parent.txPackets.pop();
                parent.canbus.send(p);
                p.timestamp = miosix::getTick();
                parent.busLoad.addPacket(p);
            }
        }

    private:
        SimpleCanManager& parent;
    };

    class CanReceiver : public Boardcore::ActiveObject
    {
    public:
        explicit CanReceiver(SimpleCanManager& parent) : parent(parent) {}
        void run() override
        {
            while (!shouldStop())
            {
                parent.canbus.getRXBuffer().waitUntilNotEmpty();
                while (!parent.canbus.getRXBuffer().isEmpty())
                {
                    Boardcore::Canbus::CanRXPacket p =
                        parent.canbus.getRXBuffer().pop();
                    parent.busLoad.addPacket(p.packet);
                    parent.rxFun(p);
                }
            }
        }

    private:
        SimpleCanManager& parent;
    };

    Boardcore::Canbus::CanbusDriver& canbus;
    Boardcore::Canbus::BusLoadEstimation busLoad;
    CanSender sender;
    CanReceiver receiver;
    RXFunction rxFun;
    Boardcore::SyncCircularBuffer<Boardcore::Canbus::CanPacket, 10> txPackets;
};
