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

#include <drivers/canbus/CanProtocol/CanProtocol.h>
#include <drivers/timer/TimestampTimer.h>
#include <scheduler/TaskScheduler.h>

using namespace std;
using namespace miosix;
using namespace Boardcore;
using namespace Canbus;

void print(const CanMessage& msg)
{
    printf("Received packet:\n");
    printf("\tpriority:       %d\n", msg.getPriority());
    printf("\tprimary type:   %d\n", msg.getPrimaryType());
    printf("\tsource:         %d\n", msg.getSource());
    printf("\tdestination:    %d\n", msg.getDestination());
    printf("\tsecondary type: %d\n", msg.getSecondaryType());
    printf("\n");
}

int main()
{
    GpioPin canA{GPIOB_BASE, 8};
    GpioPin canB{GPIOB_BASE, 9};

    canA.mode(Mode::ALTERNATE);
    canB.mode(Mode::ALTERNATE);
    canA.alternateFunction(9);
    canB.alternateFunction(9);

    GpioPin can2A{GPIOB_BASE, 12};
    GpioPin can2B{GPIOB_BASE, 13};

    can2A.mode(Mode::ALTERNATE);
    can2B.mode(Mode::ALTERNATE);
    can2A.alternateFunction(9);
    can2B.alternateFunction(9);

    printf("provolone fritto\n");

    // // Prepare the cab driver
    CanbusDriver::CanbusConfig config;
    config.loopback = true;
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = 50 * 1000;
    bitTiming.samplePoint = 87.5f / 100.0f;

    CanbusDriver* driver2 = new CanbusDriver(CAN2, config, bitTiming);

    // // Prepare the can driver
    CanProtocol protocol(driver2, print, miosix::MAIN_PRIORITY);

    // Add a filter to allow every message
    Mask32FilterBank f2(0, 0, 1, 1, 0, 0, 0);
    driver2->addFilter(f2);
    driver2->init();

    // Start the protocol
    protocol.start();

    CanMessage msg1;
    msg1.id         = 0x200;
    msg1.length     = 2;
    msg1.payload[0] = 0xffffffffffffffff;
    msg1.payload[1] = 0x0123456789ABCDEF;

    TaskScheduler scheduler;

    scheduler.addTask([&]() { protocol.enqueueMsg(msg1); }, 1000);
    scheduler.addTask(
        [&]()
        {
            PitotData data{TimestampTimer::getTimestamp(), 23};

            protocol.enqueueData(0xF, 0xA, 0x1, 0x2, 0xB, data);
        },
        2000);

    scheduler.start();

    printf("Started\n");

    while (true)
        Thread::sleep(1000);
}
