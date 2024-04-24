/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include "TestHelper.h"

#include <events/utils/EventCounter.h>
#include <utils/KernelTime.h>

#include <cmath>

using miosix::FastMutex;
using miosix::getTime;
using miosix::Lock;

namespace Boardcore
{

bool expectEvent(uint8_t eventId, uint8_t topic, long long when,
                 long long uncertainty, EventBroker& broker)
{
    EventCounter c{broker};
    c.subscribe(topic);

    long long windowStart = when - uncertainty;
    long long windowEnd   = when + uncertainty;

    while (Kernel::getOldTick() < windowEnd)
    {
        if (c.getCount(eventId) > 0)
        {
            long long recvTick = Kernel::getOldTick();
            if (recvTick < windowStart)
            {
                TRACE(
                    "[expectEvent] Event %d on topic %d receveid %d ms before "
                    "the opening of "
                    "the window.\n",
                    eventId, topic, static_cast<int>(windowStart - recvTick));
                return false;
            }
            TRACE(
                "[expectEvent] Event %d on topic %d received inside the "
                "window, %d ms from "
                "the target time.\n",
                eventId, topic, static_cast<int>(abs(recvTick - when)));

            return true;
        }

        Thread::sleep(1);
    }
    TRACE(
        "[expectEvent] The event %d on topic %d was not yet received at the "
        "end of the "
        "window.\n",
        eventId, topic);
    return false;
}

bool waitForEvent(uint8_t event, uint8_t topic, long long timeout,
                  EventBroker& broker)
{
    EventCounter c{broker};
    c.subscribe(topic);
    long long end = Kernel::getOldTick() + timeout;
    while (timeout == 0 || Kernel::getOldTick() < end)
    {
        if (c.getCount(event) > 0)
            return true;
        Thread::sleep(5);
    }
    // Timeout expired
    return false;
}

}  // namespace Boardcore
