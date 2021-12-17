/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include "test-logger.h"

#include <diagnostic/CpuMeter.h>
#include <logger/Logger.h>

using namespace Boardcore;
using namespace std;
using namespace miosix;

void logthread(void*)
{
    Logger& log      = Logger::instance();
    const int period = 5;
    for (auto t = getTick();; t += period)
    {
        Thread::sleepUntil(t);
        for (int i = 0; i < 5; i++)
        {
            Dummy d;
            d.correctValue();
            log.log(d);
        }
    }
}

void printutil(void*)
{
    for (;;)
    {
        Thread::sleep(1000);
        printf("cpu: %5.1f\n", averageCpuUtilization());
    }
}

int main()
{
    Thread::create(printutil, 4096);

    Logger& log = Logger::instance();
    log.start();

    puts("type enter to start test");
    getchar();

    Thread::create(logthread, 4096);

    puts("type enter to stop test");
    getchar();

    log.stop();

    puts("stopped");
    for (;;)
    {
        Thread::sleep(1000);
    }

    return 0;
}
