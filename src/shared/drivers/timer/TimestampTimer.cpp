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

#ifdef ENABLE_TIMESTAMP_TIMER
#ifndef COMPILE_FOR_HOST

GeneralPurposeTimer<uint32_t> TimestampTimer::timer =
    GeneralPurposeTimer<uint32_t>{TIM2};

// Create an instance of the timestamp timer to initialize and enable it
static TimestampTimer timestampTimer;

TimestampTimer::TimestampTimer()
{
    initTimestampTimer();
    enableTimestampTimer();
}

#endif
#endif

// TODO: Keep support for CortexM3
void TimestampTimer::initTimestampTimer()
{
#ifndef COMPILE_FOR_HOST
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    timer.reset();

    // Compute the timer prescaler
    uint16_t timerPrescaler = TimerUtils::computePrescalerValue(
        timer, TimestampTimer::TIMER_FREQUENCY);
    timerPrescaler--;

    timer.setPrescaler(timerPrescaler);

    // Generate an update event to apply the new prescaler value
    timer.generateUpdate();
#endif

    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Initialized timestamp timer");
}

void TimestampTimer::enableTimestampTimer()
{
#ifndef COMPILE_FOR_HOST
    timer.enable();
#endif

    PrintLogger logger = Logging::getLogger("timestamptimer");
    LOG_INFO(logger, "Enabled timestamp timer");
}

}  // namespace Boardcore
