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

#include <Singleton.h>
#include <kernel/kernel.h>

#ifndef COMPILE_FOR_HOST
#include "GeneralPurposeTimer.h"
#include "TimerUtils.h"
#endif

namespace Boardcore
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
class TimestampTimer : public Singleton<TimestampTimer>
{
    friend class Singleton<TimestampTimer>;

public:
    /**
     * @brief Preferred timer clock frequency.
     */
    static constexpr uint32_t TIMER_FREQUENCY = 250000;

    /**
     * @brief Compute the current timer value in microseconds.
     *
     * @return Current timestamp in microseconds.
     */
    uint64_t getTimestamp();

    TIM_TypeDef *getTimer();

private:
    TimestampTimer();

    /**
     * @brief Initialize the timer.
     *
     * Enables the timer clock, resets the timer registers and sets che correct
     * timer configuration.
     */
    void initTimestampTimer();

    /**
     * @brief Starts the timer peripheral.
     */
    void enableTimestampTimer();

#ifndef COMPILE_FOR_HOST
    /**
     * @brief TimestampTimer defaults to TIM2.
     */
    GeneralPurposeTimer<uint32_t> timer = GeneralPurposeTimer<uint32_t>{TIM2};
#endif
};

inline uint64_t TimestampTimer::getTimestamp()
{
#ifdef COMPILE_FOR_HOST
    return 0;
#else
    // With a timer frequency of 250KHz, the conversion from timer ticks to
    // microseconds only take a 2 byte shift (x4)
    return static_cast<uint64_t>(timer.readCounter() << 2);

    // If the timer frequency is not a multiple of 2 you must compute the value
    // this way:
    // return TimerUtils::toIntMicroSeconds(timestampTimer.getTimer());
#endif
}

}  // namespace Boardcore
