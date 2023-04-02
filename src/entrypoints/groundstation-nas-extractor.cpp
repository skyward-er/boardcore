/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

#include <drivers/usart/USART.h>
#include <mavlink_lib/pyxis/mavlink.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <scheduler/TaskScheduler.h>

using namespace miosix;
using namespace Boardcore;

using MavDriver = MavlinkDriver<20, 10>;

void sendMessage();
void receiveHandler(MavDriver*, const mavlink_message_t&);

USART usart(USART1, USARTInterface::Baudrate::B115200);
SerialTransceiver transceiver(usart);
MavDriver mavlink(&transceiver, receiveHandler);
TaskScheduler scheduler;

int main()
{
    u1rx1::mode(Mode::ALTERNATE);
    u1rx1::alternateFunction(7);
    u1tx1::mode(Mode::ALTERNATE);
    u1tx1::alternateFunction(7);

    usart.init();
    mavlink.start();
    scheduler.start();

    scheduler.addTask(sendMessage, 2000);

    while (true)
        Thread::sleep(1000);
}

void sendMessage()
{
    mavlink_message_t message;
    mavlink_msg_nas_tm_pack(1, 1, &message, TimestampTimer::getTimestamp(), 0,
                            0.75, 2.54, 120.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                            0, 0, 0);
    mavlink.enqueueMsg(message);
    printf("Sent message\n");
}

void receiveHandler(MavDriver* channel, const mavlink_message_t& msg)
{
    printf("Received message (%llu) %f %f %f\n",
           mavlink_msg_nas_tm_get_timestamp(&msg),
           mavlink_msg_nas_tm_get_nas_n(&msg),
           mavlink_msg_nas_tm_get_nas_e(&msg),
           mavlink_msg_nas_tm_get_nas_d(&msg));
}
