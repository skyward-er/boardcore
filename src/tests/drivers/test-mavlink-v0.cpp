/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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

// Ignore warnings, as we don't want to change third party generated files to
// fix them
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wcast-align"
#pragma GCC diagnostic ignored "-Waddress-of-packed-member"
#include <mavlink_lib/lynx/mavlink.h>
#pragma GCC diagnostic pop

#include <radio/MavlinkDriver/MavlinkDriverV0.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>

using namespace miosix;
using namespace Boardcore;

static const unsigned int silenceAfterSend = 250;
static const unsigned int maxPktAge        = 1000;
static const unsigned int pingPeriod       = 1000;

// Mavlink out buffer with 10 packets, 256 bytes each.
static const unsigned int queueLen   = 10;
static const unsigned int packetSize = 256;
using MavDriver =
    MavlinkDriverV0<packetSize, queueLen, MAVLINK_MAX_DIALECT_PAYLOAD_SIZE>;

SerialTransceiver* transceiver;
MavDriver* mavlink;

/**
 * @brief This test enqueues a ping message every second and replies to every
 * received message with an ACK.
 */
int main()
{
    STM32SerialWrapper serial(USART1, 19200);
    transceiver = new SerialTransceiver(serial);
    mavlink = new MavDriver(transceiver, nullptr, maxPktAge, silenceAfterSend);

    mavlink->start();

    // Send a ping every second
    while (1)
    {
        // Create a Mavlink message
        mavlink_message_t pingMsg;
        mavlink_msg_ping_tc_pack(1, 1, &pingMsg, miosix::getTick());

        // Send the message
        mavlink->enqueueMsg(pingMsg);

        miosix::Thread::sleep(pingPeriod);
    }

    return 0;
}
