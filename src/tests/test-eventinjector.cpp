/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <events/utils/EventCounter.h>
#include <events/utils/EventInjector.h>

#include "Common.h"

using namespace Boardcore;
using namespace miosix;

/**
 * The event counter is subscribed to topic 5.
 * Try to post events to that topic and check the counter.
 */
int main()
{
    const uint8_t topic = 5;

    EventInjector injector;
    injector.start();

    EventCounter counter(*sEventBroker);
    counter.subscribe({topic});

    for (;;)
    {
        printf("Last event : %d - Counter : %d \n", counter.getLastEvent(),
               counter.getCount(counter.getLastEvent()));
        Thread::sleep(5000);
    }
}