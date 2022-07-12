

/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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

#include <drivers/canbus/CanProtocol.h>

#include <thread>

#include "drivers/canbus/BusLoadEstimation.h"
#include "drivers/canbus/Canbus.h"
#include "utils/collections/CircularBuffer.h"

constexpr uint32_t BAUD_RATE         = 500 * 1000;
constexpr float SAMPLE_POINT         = 87.5f / 100.0f;
constexpr uint32_t MSG_DEADLINE      = 100;  // ms
constexpr uint32_t MSG_LOST_DEADLINE = 400;  // ms

using std::string;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace miosix;

#ifdef _ARCH_CORTEXM3_STM32
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#else
using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;
#endif

#define SLP 100
miosix::FastMutex mutex;
CanData toSend1;
CanData toSend2;
void sendData(CanProtocol* protocol, CanData* toSend)
{
    while (true)
    {

        TRACE("send\n");
        {
            miosix::Lock<miosix::FastMutex> l(mutex);
            (*protocol).sendData(*toSend);
        }
        Thread::sleep(SLP);
    }
}
bool equal(CanData* first, CanData* second)
{
    if ((*first).canId != (*second).canId ||
        (*first).length != (*second).length)
    {
        return false;
    }
    for (int i = 0; i < (*first).length; i++)
    {
        if ((*first).payload[i] != (*second).payload[i])
            return false;
    }
    return true;
}

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;

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
    CanbusDriver::CanbusConfig cfg{};
    CanbusDriver::AutoBitTiming bt;
    bt.baudRate     = BAUD_RATE;
    bt.samplePoint  = SAMPLE_POINT;
    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);
    CanProtocol protocol(c);
    // Allow every message
    Mask32FilterBank f2(0, 0, 0, 0, 0, 0, 0);

    c->addFilter(f2);
    c->init();
    protocol.start();

    toSend1.canId      = 0x200;
    toSend1.length     = 8;
    toSend1.payload[0] = 0xffffffffffffffff;
    toSend1.payload[1] = 2;
    toSend1.payload[2] = 78022;
    toSend1.payload[3] = 0xfffffffffffff;
    toSend1.payload[4] = 23;
    toSend1.payload[5] = 3234;
    toSend1.payload[6] = 12;
    toSend1.payload[7] = 0;
    std::thread firstSend(sendData, &protocol, &toSend1);

    Thread::sleep(10);
    toSend2.canId      = 0x100;
    toSend2.length     = 4;
    toSend2.payload[0] = 0xffffff;
    toSend2.payload[1] = 2;
    toSend2.payload[2] = 0x123ff;
    toSend2.payload[3] = 1;
    std::thread secondSend(sendData, &protocol, &toSend2);
    TRACE("start \n");

    for (;;)
    {
        protocol.waitBufferEmpty();
        CanData temp = protocol.getPacket();
        TRACE("received packet \n");
        if ((!equal(&temp, &toSend1) && !equal(&temp, &toSend2)))
        {
            TRACE("Error\n");
            TRACE("Received  %lu\n", temp.canId);
            TRACE("Received %d\n", temp.length);
            for (int i = 0; i < temp.length; i++)
            {
                TRACE("Received payload %d:  %llu,\n", i, temp.payload[i]);
            }
        }
        else
        {
            TRACE("OK :) id  %lu\n", temp.canId);
        }
    }
}
