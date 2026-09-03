/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Damiano Amatruda
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

#pragma once

#ifndef COMPILE_FOR_HOST
#error "Unsupported"
#endif

#include <pthread.h>

#include <cstdint>
#include <cstdlib>
#include <functional>
#include <new>

#include "scheduler/sched_types.h"

namespace miosix_private
{
class FaultData;
}

namespace miosix
{

void disableInterrupts();

void enableInterrupts();

inline void fastDisableInterrupts();

inline void fastEnableInterrupts();

class InterruptDisableLock
{
};

class InterruptEnableLock
{
public:
    InterruptEnableLock(InterruptDisableLock &l);
};

class FastInterruptDisableLock
{
};

class FastInterruptEnableLock
{
public:
    FastInterruptEnableLock(FastInterruptDisableLock &l);
};

void pauseKernel();

void restartKernel();

bool areInterruptsEnabled();

class PauseKernelLock
{
};

class RestartKernelLock
{
public:
    RestartKernelLock(PauseKernelLock &l);
};

void startKernel();

bool isKernelRunning();

long long getTime();

long long IRQgetTime();

enum class TimedWaitResult
{
    NoTimeout,
    Timeout
};

class Thread
{
public:
    enum Options
    {
        DEFAULT  = 0,
        JOINABLE = 1 << 0
    };

    static Thread *create(void *(*startfunc)(void *), unsigned int stacksize,
                          Priority priority = Priority(), void *argv = NULL,
                          unsigned short options = DEFAULT);

    static Thread *create(void (*startfunc)(void *), unsigned int stacksize,
                          Priority priority = Priority(), void *argv = NULL,
                          unsigned short options = DEFAULT);

    static void yield();

    static bool testTerminate();

    static void sleep(unsigned int ms);

    static void nanoSleep(unsigned int ns);

    static void nanoSleepUntil(long long absoluteTime);

    static Thread *getCurrentThread();

    static bool exists(Thread *p);

    Priority getPriority();

    static void setPriority(Priority pr);

    void terminate();

    static void wait();

    void wakeup();

    void PKwakeup();

    void detach();

    bool isDetached() const;

    bool join(void **result = NULL);

    static Thread *IRQgetCurrentThread();

    Priority IRQgetPriority();

    static void IRQwait();

    static TimedWaitResult IRQenableIrqAndTimedWait(
        FastInterruptDisableLock &dLock, long long absoluteTimeNs);

    static TimedWaitResult IRQenableIrqAndTimedWait(InterruptDisableLock &dLock,
                                                    long long absoluteTimeNs);

    void IRQwakeup();

    static bool IRQexists(Thread *p);

    static const unsigned int *getStackBottom();

    static int getStackSize();

    // ProcessBase *getProcess();

    static void IRQhandleSvc(unsigned int svcNumber);

    static bool IRQreportFault(const miosix_private::FaultData &fault);
};

class LowerPriority
{
public:
    bool operator()(Thread *a, Thread *b);
};

struct SleepData
{
    Thread *p;

    long long wakeup_time;

    SleepData *next;
};

}  // namespace miosix
