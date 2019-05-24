/* Copyright (c) 2017 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "CpuMeter.h"
#include <Common.h>
#include "diagnostic/SkywardStack.h"

using namespace miosix;

const int period         = 100;
const int gap            = 100;
const int watchdogPeriod = 20 * period;

static volatile float utilization   = 0.f;
static volatile unsigned int update = 0;

float averageCpuUtilization() { return utilization; }

#ifdef ENABLE_CPU_METER

static void cpuMeterThread(void*)
{
    for (;;)
    {
        long long t1 = getTick();
        delayMs(period);
        long long t2 = getTick();

        update++;
        float delta = t2 - t1;
        utilization = 100.f * (1.f - static_cast<float>(period) / delta);

        Thread::sleep(gap);

        LOG_STACK("CpuMeter");
    }
}

static void watchdogThread(void*)
{
    for (unsigned int previous = update;; previous = update)
    {
        Thread::sleep(watchdogPeriod);
        if (previous == update)
            utilization = 100.f;

        LOG_STACK("CpuWatchdog");
    }
}

class CpuMeterLauncher
{
public:
    CpuMeterLauncher()
    {
        // Create the cpu meter thread with minimum priority
        Thread::create(cpuMeterThread, skywardStack(STACK_MIN), 0, nullptr);
        Thread::create(watchdogThread, skywardStack(STACK_MIN), MAIN_PRIORITY,
                       nullptr);
    }
};

static CpuMeterLauncher launcher;

#endif  // ENABLE_CPU_METER
