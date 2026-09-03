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

#include "kernel.h"

#include "kernel/scheduler/priority/priority_scheduler_types.h"

namespace miosix
{

void disableInterrupts() {}

void enableInterrupts() {}

inline void fastDisableInterrupts() {}

inline void fastEnableInterrupts() {}

InterruptEnableLock::InterruptEnableLock(InterruptDisableLock &l) {}

FastInterruptEnableLock::FastInterruptEnableLock(FastInterruptDisableLock &l) {}

void pauseKernel() {}

void restartKernel() {}

bool areInterruptsEnabled() { return true; }

RestartKernelLock::RestartKernelLock(PauseKernelLock &l) {}

void startKernel() {}

bool isKernelRunning() { return true; }

long long getTime() { return time(NULL) * 1000; }

long long IRQgetTime() { return time(NULL) * 1000; }

Thread *Thread::create(void *(*startfunc)(void *), unsigned int stacksize,
                       Priority priority, void *argv, unsigned short options)
{
    return new Thread();
}

Thread *Thread::create(void (*startfunc)(void *), unsigned int stacksize,
                       Priority priority, void *argv, unsigned short options)
{
    return new Thread();
}

void Thread::yield() {}

bool Thread::testTerminate() { return true; }

void Thread::sleep(unsigned int ms) {}

void Thread::nanoSleep(unsigned int ns) {}

void Thread::nanoSleepUntil(long long absoluteTime) {}

Thread *Thread::getCurrentThread() { return new Thread(); }

bool Thread::exists(Thread *p) { return true; }

Priority Thread::getPriority() { return Priority{}; }

void Thread::setPriority(Priority pr) {}

void Thread::terminate() {}

void Thread::wait() {}

void Thread::wakeup() {}

void Thread::PKwakeup() {}

void Thread::detach() {}

bool Thread::isDetached() const { return false; }

bool Thread::join(void **result) { return true; }

Thread *Thread::IRQgetCurrentThread() { return new Thread(); }

Priority Thread::IRQgetPriority() { return Priority{}; }

void Thread::IRQwait() {}

TimedWaitResult Thread::IRQenableIrqAndTimedWait(
    FastInterruptDisableLock &dLock, long long absoluteTimeNs)
{
    return TimedWaitResult::NoTimeout;
}

TimedWaitResult Thread::IRQenableIrqAndTimedWait(InterruptDisableLock &dLock,
                                                 long long absoluteTimeNs)
{
    return TimedWaitResult::NoTimeout;
}

void Thread::IRQwakeup() {}

bool Thread::IRQexists(Thread *p) { return true; }

const unsigned int *Thread::getStackBottom() { return 0; }

int Thread::getStackSize() { return __INT_MAX__; }

// ProcessBase *Thread::getProcess()
// {
// }

void Thread::IRQhandleSvc(unsigned int svcNumber) {}

bool Thread::IRQreportFault(const miosix_private::FaultData &fault)
{
    return false;
}

bool LowerPriority::operator()(Thread *a, Thread *b)
{
    return a->getPriority() < b->getPriority();
}

}  // namespace miosix
