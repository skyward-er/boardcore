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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "CpuMeter.h"

#include <diagnostic/SkywardStack.h>
#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>

using namespace miosix;

namespace Boardcore
{

namespace CpuMeter
{

const int period         = 100;
const int gap            = 100;
const int watchdogPeriod = 20 * period;

static FastMutex utilizationMutex;
static Stats utilization;
static volatile unsigned int update = 0;

CpuMeterData getCpuStats()
{
    Lock<FastMutex> l(utilizationMutex);
    return CpuMeterData(TimestampTimer::getTimestamp(), utilization.getStats(),
                        MemoryProfiling::getAbsoluteFreeHeap(),
                        MemoryProfiling::getCurrentFreeHeap(),
                        MemoryProfiling::getAbsoluteFreeStack(),
                        MemoryProfiling::getCurrentFreeStack());
}

void resetCpuStats() { utilization.reset(); }

#ifdef ENABLE_CPU_METER

static void cpuMeterThread(void*)
{
    for (;;)
    {
        long long t1 = getTime() / 1e6;
        delayMs(period);
        long long t2 = getTime() / 1e6;

        update++;
        float delta = t2 - t1;
        {
            Lock<FastMutex> l(utilizationMutex);
            utilization.add(100.f * (1.f - static_cast<float>(period) / delta));
        }

        Thread::sleep(gap);

        StackLogger::getInstance().updateStack(THID_CPU_METER);
    }
}

static void watchdogThread(void*)
{
    for (unsigned int previous = update;; previous = update)
    {
        Thread::sleep(watchdogPeriod);
        if (previous == update)
        {
            Lock<FastMutex> l(utilizationMutex);
            utilization.add(100.0);
        }

        StackLogger::getInstance().updateStack(THID_CPU_WD);
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

}  // namespace CpuMeter

}  // namespace Boardcore
