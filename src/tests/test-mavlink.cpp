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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Common.h>
#include <drivers/gamma868/Gamma868.h>
#include "drivers/mavlink/MavManager.h"

using namespace miosix;

Gamma868* gamma868;
MavManager* mavManager;

static void onReceive(MavSender* sender, const mavlink_message_t& msg) 
{
    if (msg.msgid != MAVLINK_MSG_ID_ACK_TM) 
    {
        TRACE("[TmtcTest] Sending ack\n");

        mavlink_message_t ackMsg;
        mavlink_msg_ack_tm_pack(1, 1, &ackMsg, msg.msgid, msg.seq);

        /* Send the message back to the sender */
        bool ackSent = sender->enqueueMsg(ackMsg);

        if(!ackSent)
            TRACE("[Receiver] Could not enqueue ack\n");
    }
}

int main()
{
    gamma868 = new Gamma868("/dev/radio");

    mavManager = new MavManager();
    mavManager->addSender(gamma868, 250);
    mavManager->addReceiver(gamma868, mavManager->getSender(0), &onReceive);

    while(1)
    {
        TRACE("[TmtcTest] Enqueueing ping\n");

        // Create a Mavlink message
        mavlink_message_t pingMsg;
        mavlink_msg_ping_tc_pack(1, 1, &pingMsg, miosix::getTick());

        // Send the message
        bool ackSent = mavManager->getSender(0)->enqueueMsg(pingMsg);

        if(!ackSent)
            TRACE("[TmtcTest] Could not enqueue ping\n");

        // ledOn();
        // miosix::delayMs(200);
        // ledOff();

        miosix::Thread::sleep(1000);
    }

    return 0;
}
