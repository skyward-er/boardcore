/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Federico Terraneo
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

#include <Common.h>
#include <drivers/BusTemplate.h>
#include <events/Scheduler.h>
#include "drivers/stm32f2_f4_i2c.h"

using namespace miosix;

template <typename P, unsigned N>
void blinker()
{
    if (P::value())
        P::low();
    else
        P::high();
    delayMs(2);
}

int main()
{
    sEventScheduler->start();
    // Thread *ledTh=Thread::create(supercar,STACK_MIN);

    sEventScheduler->add(blinker<leds::led9, 100>, 100, "task100");
    sEventScheduler->add(blinker<leds::led8, 200>, 200, "task200");
    sEventScheduler->add(blinker<leds::led7, 500>, 500, "task500");
    sEventScheduler->add(blinker<leds::led6, 1000>, 1000, "task1000");
    sEventScheduler->add(blinker<leds::led5, 50>, 50, "task50");
    for (;;)
    {
        Thread::sleep(1000);
        auto result = sEventScheduler->getTaskStats();
        printf("--- begin ---\n");
        printf("%u tasks\n", result.size());
        for (auto& it : result)
            printf("Name: %s\n", it.name.c_str());
        printf("--- end ---\n");
    }
}
