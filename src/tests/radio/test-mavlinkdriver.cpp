/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#include <mavlink_lib/pyxis/mavlink.h>
#pragma GCC diagnostic pop

#include <drivers/usart/USART.h>
#include <miosix.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <scheduler/TaskScheduler.h>

using namespace miosix;
using namespace Boardcore;

typedef miosix::Gpio<GPIOA_BASE, 2> u2tx;
typedef miosix::Gpio<GPIOA_BASE, 3> u2rx;

USART usart(USART2, 115200);
SerialTransceiver transceiver(usart);
MavlinkDriver<20, 10> mavlink(&transceiver);

void payloadGenerator();

int main()
{
    u2rx::mode(Mode::ALTERNATE);
    u2rx::alternateFunction(7);
    u2tx::mode(Mode::ALTERNATE);
    u2tx::alternateFunction(7);

    TaskScheduler scheduler;
    scheduler.addTask(payloadGenerator, 2000);
    scheduler.start();

    usart.init();
    mavlink.start();

    while (true)
        Thread::sleep(1000);
}

void payloadGenerator()
{
    // uint8_t msg[] = "012345678";
    // mavlink.enqueueRaw(msg, 9);
    // printf("Added 9 bytes: %s\n", msg);

    // Create a mavlink packet
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(0x01, 0x23, &ackMsg, 0x45, 0x67);
    mavlink.enqueueMsg(ackMsg);
}
