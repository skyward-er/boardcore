/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Davide Mor, Alberto Nidasio
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

#include <utils/ClockUtils.h>

namespace Boardcore
{

/**
 * @brief Timer utilities.
 */
namespace TimerUtils
{

/**
 * @brief Returns the timer input clock.
 *
 * @return Timer input clock, APB1 or ABP2.
 */
ClockUtils::APB getTimerInputClock(TIM_TypeDef *timer);

/**
 * @brief Returns the timer clock frequency before the prescaler.
 *
 * Class borrowed from the SyncronizedServo class in Miosix.
 *
 * @param timer Timer to use.
 * @return Prescaler input frequency.
 */
uint32_t getPrescalerInputFrequency(TIM_TypeDef *timer);

/**
 * @brief Return the timer clock frequency.
 *
 * @param timer Timer to use.
 * @return Timer frequency.
 */
uint32_t getFrequency(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in microseconds.
 */
float toMicroSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in microseconds.
 */
float toMicroSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * Calculation using integer values.
 *
 * @returns Timer counter in microseconds.
 */
uint64_t toIntMicroSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * Calculation using integer values.
 *
 * @returns Timer counter in microseconds.
 */
uint64_t toIntMicroSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the specified value converted in milliseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in milliseconds.
 */
float toMilliSeconds(TIM_TypeDef *timer, uint32_t value);

/**
 * @brief Returns the timer counter converted in milliseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in milliseconds.
 */
float toMilliSeconds(TIM_TypeDef *timer);

/**
 * @brief Returns the timer counter converted in seconds based on the timer
 * clock frequency and prescaler.
 *
 * @returns Timer counter in seconds.
 */
float toSeconds(TIM_TypeDef *timer);

/**
 * @brief Computes the timer resolution in microseconds.
 *
 * @return Microseconds per timer tick.
 */
float getResolution(TIM_TypeDef *timer);

/**
 * @brief Computes the number of seconds for timer reset.
 *
 * @return Timer duration before counter reset in secondss.
 */
float getMaxDuration(TIM_TypeDef *timer);

/**
 * @brief Compute the prescaler value for the specified target frequency.
 *
 * @return Prescaler value for the target frequency.
 */
uint16_t computePrescalerValue(TIM_TypeDef *timer, int targetFrequency);

}  // namespace TimerUtils

inline ClockUtils::APB TimerUtils::getTimerInputClock(TIM_TypeDef *timer)
{
    // Timers can be connected to APB1 or APB2 clocks.
    // APB1: TIM2-7,12-15
    // APB2: TIM1,8-11
    // TODO: Add support for F103
    if (timer == TIM1 || timer == TIM8 || timer == TIM9 || timer == TIM10 ||
        timer == TIM11)
    {
        return ClockUtils::APB::APB2;
    }
    else
    {
        return ClockUtils::APB::APB1;
    }
}

inline uint32_t TimerUtils::getPrescalerInputFrequency(TIM_TypeDef *timer)
{
    return ClockUtils::getAPBFrequecy(getTimerInputClock(timer));
}

inline uint32_t TimerUtils::getFrequency(TIM_TypeDef *timer)
{
    return getPrescalerInputFrequency(timer) / (1 + timer->PSC);
}

inline float TimerUtils::toMicroSeconds(TIM_TypeDef *timer, uint32_t value)
{
    return (1.0f * value * 1e6 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::toMicroSeconds(TIM_TypeDef *timer)
{
    return toMicroSeconds(timer, timer->CNT);
}

inline uint64_t TimerUtils::toIntMicroSeconds(TIM_TypeDef *timer,
                                              uint32_t value)
{
    return ((uint64_t)value * 1e6 * (uint64_t)(1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline uint64_t TimerUtils::toIntMicroSeconds(TIM_TypeDef *timer)
{
    return toIntMicroSeconds(timer, timer->CNT);
}

inline float TimerUtils::toMilliSeconds(TIM_TypeDef *timer, uint32_t value)
{
    return (1.0f * value * 1e3 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::toMilliSeconds(TIM_TypeDef *timer)
{
    return toMilliSeconds(timer, timer->CNT);
}

inline float TimerUtils::toSeconds(TIM_TypeDef *timer)
{
    return (1.0f * timer->CNT * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::getResolution(TIM_TypeDef *timer)
{
    return (1.0e6f * (1 + timer->PSC)) / getPrescalerInputFrequency(timer);
}

inline float TimerUtils::getMaxDuration(TIM_TypeDef *timer)
{
    return (1.0f * timer->ARR * 1e6 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline uint16_t TimerUtils::computePrescalerValue(TIM_TypeDef *timer,
                                                  int targetFrequency)
{
    return TimerUtils::getPrescalerInputFrequency(timer) / targetFrequency - 1;
}

}  // namespace Boardcore
