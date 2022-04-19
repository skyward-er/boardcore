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

TimestampTimer::TimestampTimer()
{
    initTimestampTimer();
    enableTimestampTimer();
}

#ifndef COMPILE_FOR_HOST

TIM_TypeDef *TimestampTimer::getTimer() { return timer.getTimer(); }

void TimestampTimer::resetTimestamp() { timer.setCounter(0); }

// TODO: Keep support for STM32F103
void TimestampTimer::initTimestampTimer()
{
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    timer.reset();

    timer.setFrequency(TIMER_FREQUENCY);

    // Generate an update event to apply the new prescaler value
    timer.generateUpdate();

    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Initialized timestamp timer");
}

void TimestampTimer::enableTimestampTimer()
{
    timer.enable();

    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Enabled timestamp timer");
}

#else

void TimestampTimer::resetTimestamp() {}

void TimestampTimer::initTimestampTimer()
{
    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Initialized timestamp timer [COMPILE_FOR_HOST]");
}

void TimestampTimer::enableTimestampTimer()
{
    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Enabled timestamp timer [COMPILE_FOR_HOST]");
}

#endif

}  // namespace Boardcore
