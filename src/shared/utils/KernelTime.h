/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <miosix.h>

#include "TimeUtils.h"

namespace Boardcore
{

namespace Kernel
{

/**
 * @brief Get the current time in milliseconds.
 *
 * Calls miosix::getTime() and converts the result to milliseconds.
 * This function is not safe to call from an IRQ context.
 *
 * @deprecated Use miosix::getTime() instead and migrate to using nanoseconds.
 */
inline long long getOldTick() { return nsToMs(miosix::getTime()); }

/**
 * @brief Get the current time in milliseconds.
 *
 * Calls miosix::IRQgetTime() and converts the result to milliseconds.
 * This function is safe to call from an IRQ context.
 *
 * @deprecated Use miosix::getTime() instead and migrate to using nanoseconds.
 */
inline long long IRQgetOldTick() { return nsToMs(miosix::IRQgetTime()); }

namespace Thread
{

/**
 * @brief Sleep until a given time in milliseconds.
 *
 * Converts the given time in milliseconds to nanoseconds and calls
 * miosix::Thread::nanoSleepUntil().
 *
 * @param absoluteTimeMs The timestamp in milliseconds to sleep until.
 *
 * @deprecated Use miosix::Thread::nanoSleepUntil() instead and migrate to using
 * nanoseconds.
 */
inline void sleepUntil(long long absoluteTimeMs)
{
    miosix::Thread::nanoSleepUntil(msToNs(absoluteTimeMs));
}

/**
 * @brief Stops the thread until wakeup() is called or the specified absolute
 * time in milliseconds is reached.
 *
 * Converts the given time in milliseconds to nanoseconds and calls
 * miosix::Thread::IRQenableIrqAndTimedWait().
 *
 * @param absoluteTimeMs Absolute time after which the wait times out
 * @return The result of the wait
 *
 * @deprecated Use miosix::Thread::IRQenableIrqAndTimedWait() instead and
 * migrate to using nanoseconds.
 */
inline miosix::TimedWaitResult IRQenableIrqAndTimedWaitMs(
    miosix::FastInterruptDisableLock& dLock, long long absoluteTimeMs)
{
    return miosix::Thread::IRQenableIrqAndTimedWait(dLock,
                                                    msToNs(absoluteTimeMs));
}

}  // namespace Thread

}  // namespace Kernel

}  // namespace Boardcore
