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

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/canbus/BusLoadEstimation.h>
#include <drivers/canbus/Canbus.h>

#include <string>

using std::string;
using namespace Boardcore;
using namespace Canbus;
using namespace miosix;

using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;

PrintLogger l                = Logging::getLogger("main");
constexpr uint32_t BAUD_RATE = 1000 * 1000;

void printPacket(PrintLogger& logger, string name, Canbus::CanPacket p)
{
    LOG_INFO(logger, "Packet {}: {{id: {} ({}), len: {}, b0: {:#04X} }}", name,
             p.id, p.ext, p.length, p.data[0]);
}

void printRXPacket(PrintLogger& logger, string name, Canbus::CanRXPacket p)
{
    printPacket(logger, name, p.packet);
    LOG_DEBUG(logger,
              "RX result: fifo={}, status={:#04X}, errCnt:{}, errCode:{:#04X}",
              p.status.fifo, p.status.rxStatus, p.status.rxErrCounter,
              p.status.errCode);
}

int main()
{
    Logging::startAsyncLogger();

    PrintLogger lr = l.getChild("rx");

    {
        miosix::FastInterruptDisableLock dLock;

        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);

        CanRX::alternateFunction(9);
        CanTX::alternateFunction(9);
    }

    CanbusDriver::CanbusConfig cfg;
    cfg.loopback = true;

    CanbusDriver::AutoBitTiming bt;
    bt.baudRate    = BAUD_RATE;
    bt.samplePoint = 87.5f / 100.0f;

    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);

    // Canbus::Mask32Filter f1(365854720, 0xFF000000, 1, 1, 0, 0, 1);
    // Canbus::Mask32Filter f2(1, 1, 1, 1, 0, 0, 0);

    // Canbus::ID32Filter f(0);
    // f.addID(365854720, 1, 0);
    // f.addID(1, 1, 0);

    // Canbus::ID32Filter f1(1);
    // f1.addID(499908608, 1, 0);
    // f1.addID(0, 1, 0);

    Canbus::ID16FilterBank f1(0);
    f1.addID32(365854720, 1, 0);
    f1.addID32(500072448, 1, 0);
    // f1.addID(368836608, 1, 0);

    // Canbus::Mask16Filter f2(1);
    // f2.addID(365854720, 163840, 1,1,0,0);

    c->addFilter(f1);
    // c->addFilter(f2);

    c->init();

    CanPacket p;
    p.ext     = true;
    p.length  = 1;
    p.data[0] = 123;

    for (;;)
    {
        p.timestamp = miosix::getTick();

        p.id = 365854720;
        c->send(p);

        p.id = 500072448;
        c->send(p);

        p.id = 500085835;
        c->send(p);

        p.id = 368836608;
        c->send(p);

        p.id = 9876;
        c->send(p);

        p.id = 1;
        c->send(p);

        p.id = 3;
        c->send(p);

        p.id = 0;
        c->send(p);

        while (!c->getRXBuffer().isEmpty())
        {
            Canbus::CanRXPacket rxp = c->getRXBuffer().pop();
            printRXPacket(lr, "RX", rxp);
        }

        Thread::sleep(1000);
    }

    for (;;)
    {
        LOG_DEBUG(l, "End.");
        Thread::sleep(10000);
    }
}
