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

#pragma once

#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/TimerUtils.h>
#include <kernel/kernel.h>

namespace timer
{

/**
 * @brief Utility for precise timestamp values.
 *
 * TimestanTimer works with 32bit timers, TIM2 or TIM5.
 *
 * TIM2 is used, if we'll need to use TIM5 or both works need to be done to
 * implement timer selection.
 *
 * The preferred configuration is to use a timer frequency of 250KHz. This way
 * the timer has a resolution of 4us, a reload timer of 4.7 hours and for tick
 * to time conversion require only a shift operation.
 *
 * For timer resolution and duration refer to :
 * https://docs.google.com/spreadsheets/d/1FiNDVU7Rg98yZzz1dZ4GDAq3-nEg994ziezCawJ-OK4/edit?usp=sharing
 */
class TimestampTimer
{
public:
    /**
     * @brief TimestampTimer defaults to TIM2.
     */
    static GeneralPurposeTimer<uint32_t> timestampTimer;

    /**
     * @brief Preferred timer clock frequency.
     */
    static constexpr uint32_t TIMER_FREQUENCY = 250000;

    /**
     * @brief Initialize the timer.
     *
     * Enables the timer clock, resets the timer registers and sets che correct
     * timer configuration.
     */
    static void initTimestampTimer();

    /**
     * @brief Starts the timer peripheral.
     */
    static void enableTimestampTimer();

    /**
     * @brief Compute the current timer value in microseconds.
     *
     * @return Current timestamp in microseconds.
     */
    static uint64_t getTimestamp();
};

}  // namespace timer

// TODO: Keep support for CortexM3
inline void timer::TimestampTimer::initTimestampTimer()
{
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    }

    timestampTimer.reset();

    // TODO: mode this calculation inside TimerUtils
    // Compute the timer prescaler
    uint16_t timerPrescaler =
        TimerUtils::getPrescalerInputFrequency(timestampTimer.getTimer()) /
        TimestampTimer::TIMER_FREQUENCY;
    timerPrescaler--;

    timestampTimer.setPrescaler(timerPrescaler);

    // Generate an update event to apply the new prescaler value
    timestampTimer.generateUpdate();
}

inline void timer::TimestampTimer::enableTimestampTimer()
{
    timestampTimer.enable();
}

inline uint64_t timer::TimestampTimer::getTimestamp()
{
    // With a timer frequency of 250KHz, the conversion from timer ticks to
    // microseconds only take a 2 byte shift (x4)
    return (uint64_t)timestampTimer.readCounter() << 2;

    // If the timer frequency is not a multiple of 2 you must compute the value
    // this way:
    // return TimerUtils::toIntMicroSeconds(timestampTimer.getTimer());
}