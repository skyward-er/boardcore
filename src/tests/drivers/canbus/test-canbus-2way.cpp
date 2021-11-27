/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

// Sends packets on the canbus and waits for a response, measuring the response
// time. If a response takes more than MSG_LOST_DEADLINE, the packet is
// considered lost. It may also receive requests from other canbus devices, to
// which it will respond

#include <string>

#include "ActiveObject.h"
#include "Debug.h"
#include "SimpleCanManager.h"
#include "diagnostic/PrintLogger.h"
#include "drivers/canbus/BusLoadEstimation.h"
#include "drivers/canbus/Canbus.h"
#include "math/Stats.h"
#include "utils/collections/CircularBuffer.h"

constexpr uint32_t BAUD_RATE         = 1000 * 1000;
constexpr float SAMPLE_POINT         = 87.5f / 100.0f;
constexpr uint32_t MSG_DEADLINE      = 100;  // ms
constexpr uint32_t MSG_LOST_DEADLINE = 400;  // ms

using std::string;
using namespace Canbus;
using namespace miosix;

#ifdef _ARCH_CORTEXM3_STM32
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#else
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#endif

SimpleCanManager* can_mgr;

struct CanMsg
{
    uint32_t id;
    uint32_t ts;
};

CircularBuffer<CanMsg, 4000> msgs;
FastMutex mutex_msgs;

PrintLogger l = Logging::getLogger("main");

void handleCanMessage(Canbus::CanRXPacket packet)
{
    if (packet.packet.data[0] == 0x44)  // This is a request
    {
        CanPacket response = packet.packet;
        response.data[0]   = 0x55;
        can_mgr->send(response);
    }
    else if (packet.packet.data[0] == 0x55)  // This is a response to a request
    {
        Lock<FastMutex> l(mutex_msgs);
        for (size_t i = 0; i < msgs.count(); ++i)
        {
            CanMsg& msg = msgs.get(i);
            if (msg.id == packet.packet.id)
            {
                msg.id = 0;
                msg.ts = getTick() - msg.ts;
                break;
            }
        }
    }
}

int seq = 1;
void sendNewRequest()
{
    CanPacket packet;
    packet.id = seq++;

    if (packet.id % 2 == 0)
    {
        packet.id = 0xFFFFFFFF - packet.id;
    }

    packet.ext    = 1;
    packet.length = 8;
    packet.rtr    = false;

    packet.data[0] = 0x44;
    for (int i = 1; i < 8; ++i)
    {
        packet.data[i] = seq + i;
    }

    {
        Lock<FastMutex> l(mutex_msgs);
        msgs.put({packet.id, (uint32_t)getTick()});
    }

    can_mgr->send(packet);
}

class MessageCollector : public ActiveObject
{
public:
    void run() override
    {
        unsigned int c = 0;
        while (!shouldStop())
        {
            uint32_t tick = (uint32_t)getTick();
            {
                Lock<FastMutex> l(mutex_msgs);
                if (msgs.isFull())
                {
                    ++buffer_full;
                }
                while (!msgs.isEmpty())
                {
                    CanMsg msg = msgs.get();
                    if (msg.id == 0 || tick - msg.ts > MSG_LOST_DEADLINE)
                    {
                        msgs.pop();

                        ++total_packets;
                        if (msg.id == 0)
                        {
                            msgstats.add(msg.ts * 1.0f);
                            if (msg.ts > MSG_DEADLINE)
                            {
                                ++missed_deadline;
                            }
                        }
                        else
                        {
                            // No response
                            ++lost_packets;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
            }
            if (c % 10 == 0)
            {
                StatsResult res = msgstats.getStats();

                TRACE(
                    "Total packets: %u, Missed deadlines: %u, Lost packets: "
                    "%u, Mean ping: %.2f, "
                    "Max ping: %.0f, Min ping: %.0f, Buffer full: %u\n",
                    total_packets, missed_deadline, lost_packets, res.mean,
                    res.maxValue, res.minValue, buffer_full);

                Canbus::BusLoadEstimation::BusLoadInfo info =
                    can_mgr->getLoadSensor().getLoadInfo();
                TRACE(
                    "Payload rate: %.2f kbps, Frame rate: %.2f kbps, Load: "
                    "%.2f %%\n",
                    info.payload_bit_rate / 1000.0f,
                    info.total_bit_rate / 1000.0f, info.load_percent);
            }
            ++c;
            Thread::sleepUntil(tick + MSG_DEADLINE);
        }
    }

private:
    unsigned int total_packets   = 0;
    unsigned int lost_packets    = 0;
    unsigned int missed_deadline = 0;
    unsigned int buffer_full     = 0;
    Stats msgstats;
};

MessageCollector mc;

int main()
{
    Logging::startAsyncLogger();

    {
        miosix::FastInterruptDisableLock dLock;

        RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;  // Enable CAN1 bus

#ifdef _ARCH_CORTEXM3_STM32
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);
#else
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);

        CanRX::alternateFunction(9);
        CanTX::alternateFunction(9);
#endif
    }

    Canbus::Canbus::CanbusConfig cfg{};
    Canbus::Canbus::AutoBitTiming bt;
    bt.baud_rate    = BAUD_RATE;
    bt.sample_point = SAMPLE_POINT;

    Canbus::Canbus* c = new Canbus::Canbus(CAN1, cfg, bt);
    can_mgr           = new SimpleCanManager(*c, BAUD_RATE, handleCanMessage);

    // Allow every message
    Canbus::Mask32Filter f2(0, 0, 0, 0, 0, 0, 0);
    c->addFilter(f2);
    c->init();

    can_mgr->start();
    mc.start();

    for (;;)
    {
        sendNewRequest();
        sendNewRequest();
        sendNewRequest();
        Thread::sleep(2);
    }
}