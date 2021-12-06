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

#include <scheduler/TaskScheduler.h>

#include "Common.h"

using namespace Boardcore;
using namespace miosix;

void task5hz()
{
    static long long last_tick = getTick();

    printf("%d: 5 Hz tick\n", (int)(getTick() - last_tick));
}

void task2hz()
{
    static long long last_tick = getTick();

    printf("%d: 2 Hz tick\n", (int)(getTick() - last_tick));
}

int main()
{
    TaskScheduler scheduler;

    TaskScheduler::function_t f5hz{&task5hz};
    TaskScheduler::function_t f2hz{&task2hz};
    scheduler.add(f5hz, 1000 / 5, 1);
    scheduler.add(f2hz, 1000 / 2, 1);

    scheduler.start();

    Thread::sleep(10000);

    scheduler.stop();

    for (;;)
    {
        printf("end\n");
        Thread::sleep(5000);
    }
}