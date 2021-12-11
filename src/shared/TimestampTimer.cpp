/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Davide Mor
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

#include <Debug.h>

namespace Boardcore
{

namespace TimestampTimer
{

#ifndef COMPILE_FOR_HOST

#ifdef _ARCH_CORTEXM3_STM32
HardwareTimer<uint32_t, TimerMode::Chain> initHardwareTimer()
{
    // VERY IMPORTANT! ALWAYS ENABLE CLOCKS BEFORE CONFIGURING THE TIMERS!!
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 + TIM3 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN;
    }

    // chain two 16-bits timers
    return HardwareTimer<uint32_t, TimerMode::Chain>(
        TIM2, TIM3, TimerTrigger::ITR1,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));
}

HardwareTimer<uint32_t, TimerMode::Chain> timestamp_timer = initHardwareTimer();
#else
HardwareTimer<uint32_t, TimerMode::Single> initHardwareTimer()
{
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    return HardwareTimer<uint32_t, TimerMode::Single>(
        TIM2,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));
}

HardwareTimer<uint32_t, TimerMode::Single> timestamp_timer =
    initHardwareTimer();
#endif

#endif  // COMPILE_FOR_HOST

void enableTimestampTimer(uint8_t prescaler)
{
#ifndef COMPILE_FOR_HOST
    timestamp_timer.setPrescaler(prescaler);
    timestamp_timer.start();
#endif
}

}  // namespace TimestampTimer

}  // namespace Boardcore
