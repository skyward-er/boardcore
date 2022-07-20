/* Copyright (c) 2020-2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Davide Mor, Alberto Nidasio
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

#include "TimestampTimer.h"

#include <diagnostic/PrintLogger.h>

namespace Boardcore
{

#ifndef COMPILE_FOR_HOST

namespace TimestampTimer
{

GP32bitTimer timestampTimer = initTimestampTimer();

}  // namespace TimestampTimer

void TimestampTimer::resetTimestamp() { timestampTimer.setCounter(0); }

// TODO: Keep support for STM32F103
GP32bitTimer TimestampTimer::initTimestampTimer()
{
    GP32bitTimer timer = GP32bitTimer{TIM2};
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    timer.reset();

    timer.setFrequency(TIMER_FREQUENCY);

    // Generate an update event to apply the new prescaler value
    timer.generateUpdate();

    timestampTimer.enable();

    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Initialized timestamp timer");

    return timer;
}

#else

void TimestampTimer::resetTimestamp() {}

#endif

}  // namespace Boardcore
