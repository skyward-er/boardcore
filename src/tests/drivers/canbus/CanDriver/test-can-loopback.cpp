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
#include <drivers/canbus/CanDriver/BusLoadEstimation.h>
#include <drivers/canbus/CanDriver/CanDriver.h>

#include <string>

using std::string;
using namespace Boardcore;
using namespace Boardcore::Canbus;
using namespace miosix;

using CanRX = Gpio<GPIOA_BASE, 11>;
using CanTX = Gpio<GPIOA_BASE, 12>;

PrintLogger l                = Logging::getLogger("main");
constexpr uint32_t BAUD_RATE = 1000 * 1000;

void printPacket(string name, CanPacket p)
{
    LOG_INFO(l, "Packet {}: {{id: {} ({}), len: {}, b0: {:#04X} }}", name, p.id,
             p.ext, p.length, p.data[0]);
}

void printRXPacket(string name, CanRXPacket p)
{
    printPacket(name, p.packet);
    LOG_DEBUG(l, "RX result: status={:#04X}, rxErrCounter:{}, errCode:{:#04X}",
              p.status.rxStatus, p.status.rxErrCounter, p.status.errCode);
}

class BusLoadSensor : public ActiveObject
{
public:
    explicit BusLoadSensor(uint32_t baudRate)
        : baudRate(baudRate), ble(baudRate)
    {
    }

    void addPacket(CanPacket p) { ble.addPacket(p); }
    void run() override
    {
        while (!shouldStop())
        {
            long long start                     = miosix::IRQgetTime() / 1e6;
            BusLoadEstimation::BusLoadInfo info = ble.getLoadInfo();
            LOG_INFO(l,
                     "payload rate: {:.2f} kbps, total rate: {:.2f} kbps, "
                     "utilization: {:.2f}%",
                     info.payloadBitRate / 1000.0f, info.totalBitRate / 1000.0f,
                     info.loadPercent);
            Thread::sleepUntil(start + 1000);
        }
    }

private:
    PrintLogger l = Logging::getLogger("busLoad");
    uint32_t baudRate;
    BusLoadEstimation ble;
};

BusLoadSensor load(BAUD_RATE);

int main()
{
    Logging::startAsyncLogger();

    {
        miosix::FastInterruptDisableLock dLock;

        CanRX::mode(Mode::ALTERNATE);
        CanTX::mode(Mode::ALTERNATE);

        CanRX::alternateFunction(9);
        CanTX::alternateFunction(9);
    }

    load.start();

    CanbusDriver::CanbusConfig cfg;
    cfg.loopback = true;

    CanbusDriver::AutoBitTiming bt;
    bt.baudRate    = BAUD_RATE;
    bt.samplePoint = 87.5f / 100.0f;

    CanbusDriver* c = new CanbusDriver(CAN1, cfg, bt);

    Mask32FilterBank f2(0, 0, 0, 0, 0, 0, 0);

    c->addFilter(f2);
    c->init();
    CanPacket p;
    p.id     = 12345;
    p.ext    = true;
    p.length = 8;
    for (int i = 0; i < p.length; ++i)
        p.data[i] = 1 << i;
    for (;;)
    {

        // printPacket("TX", p);
        p.timestamp = miosix::IRQgetTime() / 1e6;
        c->send(p);
        load.addPacket(p);
        // Thread::sleep(1);
        // c->send(p);
        // c->send(p);
        // c->send(p);
        // c->send(p);

        while (!c->getRXBuffer().isEmpty())
        {
            load.addPacket(c->getRXBuffer().pop().packet);
        }
        // c->getRXBuffer().waitUntilNotEmpty();
        // CanRXPacket prx = c->getRXBuffer().pop();
        // printRXPacket("RX", prx);

        // c->getTXResultBuffer().waitUntilNotEmpty();
        // CanTXResult resTx = c->getTXResultBuffer().pop();
        // LOG_DEBUG(l,
        //           "TX result: mailbox={}, status={:#04X}, tme={:#04X}, "
        //           "errCode={:#04X}",
        //           resTx.mailbox, resTx.txStatus, resTx.tme,
        //           resTx.errCode);

        Thread::sleep(1);
    }

    for (;;)
    {
        LOG_DEBUG(l, "End.");
        Thread::sleep(10000);
    }
}
