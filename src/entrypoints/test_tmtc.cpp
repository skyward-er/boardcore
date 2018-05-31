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
#define DEBUG
#include "boards/Homeone/TMTCManager/TMTCManager.h"

using namespace miosix;
using namespace HomeoneBoard;
using namespace TMTC;

int main()
{

    while(1)
    {
#ifdef DEBUG
        printf("Enqueuing ping\n");
#endif

        // Create a Mavlink message
        mavlink_message_t pingMsg;
        uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];

        // Populate Mavlink message passing the parameters of the specific message
        mavlink_msg_ping_tc_pack(1, 1, &pingMsg, miosix::getTick());

        // Convert it into a byte stream
        int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &pingMsg);

        // Send the message
        bool ackSent = sTMTCManager->enqueueMsg(bufferMsg, msgLen);

        ledOn();
        miosix::delayMs(200);
        ledOff();

        miosix::delayMs(5000);
    }

    return 0;
}
