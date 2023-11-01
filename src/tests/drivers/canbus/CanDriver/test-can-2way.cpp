/* Copyright (c) 2021 Skyward Experimental Rocketry
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

// Sends packets on the canbus and waits for a response, measuring the response
// time. If a response takes more than MSG_LOST_DEADLINE, the packet is
// considered lost. It may also receive requests from other canbus devices, to
// which it will respond

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/canbus/CanDriver/BusLoadEstimation.h>
#include <drivers/canbus/CanDriver/CanDriver.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>
#include <utils/collections/CircularBuffer.h>

#include <string>

#include "SimpleCanManager.h"

constexpr uint32_t BAUD_RATE         = 500 * 1000;
constexpr float SAMPLE_POINT         = 87.5f / 100.0f;
constexpr uint32_t MSG_DEADLINE      = 100;  // ms
constexpr uint32_t MSG_LOST_DEADLINE = 400;  // ms

using std::string;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace miosix;

#ifdef _ARCH_CORTEXM3_STM32F1
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#else
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#endif

SimpleCanManager* canManager;

struct CanMsg
{
    uint32_t id;
    uint32_t ts;
};

CircularBuffer<CanMsg, 4000> msgs;
FastMutex mutexMsgs;

void handleCanMessage(Canbus::CanRXPacket packet)
{
    if (packet.packet.data[0] == 0x44)  // This is a request
    {
        CanPacket response = packet.packet;
        response.data[0]   = 0x55;
        canManager->send(response);
    }
    else if (packet.packet.data[0] == 0x55)  // This is a response to a request
    {
        Lock<FastMutex> l(mutexMsgs);

        for (size_t i = 0; i < msgs.count(); ++i)
        {
            CanMsg& msg = msgs.get(i);
            if (msg.id == packet.packet.id)
            {
                msg.id = 0;
                msg.ts = getTime() / 1e6 - msg.ts;
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
        packet.id = 0xFFFFFFF - packet.id;
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
        Lock<FastMutex> l(mutexMsgs);
        msgs.put({packet.id, (uint32_t)getTime() / 1e6});
    }

    canManager->send(packet);
}

class MessageCollector : public ActiveObject
{
public:
    void run() override
    {
        unsigned int c = 0;
        while (!shouldStop())
        {
            uint32_t tick = (uint32_t)getTime() / 1e6;
            {
                Lock<FastMutex> l(mutexMsgs);
                if (msgs.isFull())
                {
                    ++bufferFull;
                }
                while (!msgs.isEmpty())
                {
                    CanMsg msg = msgs.get();
                    if (msg.id == 0 || tick - msg.ts > MSG_LOST_DEADLINE)
                    {
                        msgs.pop();

                        ++totalPackets;
                        if (msg.id == 0)
                        {
                            msgstats.add(msg.ts * 1.0f);
                            if (msg.ts > MSG_DEADLINE)
                            {
                                ++missedDeadline;
                            }
                        }
                        else
                        {
                            // No response
                            ++lostPackets;
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
                StatsResult res __attribute__((unused)) = msgstats.getStats();

                printf(
                    "Total packets: %u, Missed deadlines: %u, Lost packets: "
                    "%u, Mean ping: %.2f, "
                    "Max ping: %.0f, Min ping: %.0f, Buffer full: %u\n",
                    totalPackets, missedDeadline, lostPackets, res.mean,
                    res.maxValue, res.minValue, bufferFull);

                Canbus::BusLoadEstimation::BusLoadInfo info
                    __attribute__((unused)) =
                        canManager->getLoadSensor().getLoadInfo();
                printf(
                    "Payload rate: %.2f kbps, Frame rate: %.2f kbps, Load: "
                    "%.2f %%\n",
                    info.payloadBitRate / 1000.0f, info.totalBitRate / 1000.0f,
                    info.loadPercent);
            }
            ++c;
            Thread::nanoSleepUntil(tick + MSG_DEADLINE);
        }
    }

private:
    unsigned int totalPackets   = 0;
    unsigned int lostPackets    = 0;
    unsigned int missedDeadline = 0;
    unsigned int bufferFull     = 0;
    Stats msgstats;
};

MessageCollector mc;

int main()
{
    Logging::startAsyncLogger();

    {
        miosix::FastInterruptDisableLock dLock;

#ifdef _ARCH_CORTEXM3_STM32F1
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);
#else
        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);

        CanRX::alternateFunction(9);
        CanTX::alternateFunction(9);
#endif
    }

    CanbusDriver::CanbusConfig cfg{};
    CanbusDriver::AutoBitTiming bt;
    bt.baudRate    = BAUD_RATE;
    bt.samplePoint = SAMPLE_POINT;

    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);
    canManager      = new SimpleCanManager(*c, BAUD_RATE, handleCanMessage);

    // Allow every message
    Mask32FilterBank f2(0, 0, 0, 0, 0, 0, 0);
    c->addFilter(f2);
    c->init();

    canManager->start();
    mc.start();
    const int slp = 5;
    for (;;)
    {
        sendNewRequest();
        Thread::sleep(slp);
        sendNewRequest();
        Thread::sleep(slp);
        sendNewRequest();
        Thread::sleep(slp);
        sendNewRequest();
        Thread::sleep(slp);
    }
}
