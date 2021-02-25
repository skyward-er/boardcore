/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include "TimestampTimer.h"

namespace TimestampTimer
{

HardwareTimer<uint32_t> timestamp_timer{
    TIM2, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

void enableTimestampTimer()
{

    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    timestamp_timer.setPrescaler(PRESCALER_VALUE);
    timestamp_timer.setAutoReload(RELOAD_VALUE);

    timestamp_timer.start();
}

}  // namespace TimestampTimer