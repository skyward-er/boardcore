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

#include <interfaces/arch_registers.h>

namespace Boardcore
{

/**
 * @brief Timer utilities.
 */
namespace TimerUtils
{

/**
 * @brief Timer input clock.
 */
enum class InputClock
{
    APB1,
    APB2
};

/**
 * @brief Returns the timer input clock.
 *
 * @return Timer input clock, APB1 or ABP2.
 */
InputClock getTimerInputClock(TIM_TypeDef *timer);

/**
 * @brief Returns the timer clock frequency before the prescaler.
 *
 * Class borrowed from the SyncronizedServo class in Miosix.
 *
 * @param inputClock Timer input clock.
 * @return Prescaler input frequency.
 */
uint32_t getPrescalerInputFrequency(InputClock inputClock);

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
 * @brief Returns the timer counter converted in microseconds based on the
 * timer clock frequency and prescaler.
 *
 * @returns Timer counter in microseconds.
 */
float toMicroSeconds(TIM_TypeDef *timer);

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
 * @brief Computer the prescaler value for the specified target frequency.
 *
 * @return Prescaler value for the target frequency.
 */
uint16_t computePrescalerValue(TIM_TypeDef *timer, int targetFrequency);

}  // namespace TimerUtils

inline TimerUtils::InputClock TimerUtils::getTimerInputClock(TIM_TypeDef *timer)
{
    // Timers can be connected to APB1 or APB2 clocks.
    // APB1: TIM2-7,12-15
    // APB2: TIM1,8-11
    if (timer == TIM1 || timer == TIM8 || timer == TIM9 || timer == TIM10 ||
        timer == TIM11)
    {
        return InputClock::APB2;
    }
    else
    {
        return InputClock::APB1;
    }
}

inline uint32_t TimerUtils::getPrescalerInputFrequency(InputClock inputClock)
{
    // The global variable SystemCoreClock from ARM's CMIS allows to know the
    // CPU frequency.
    uint32_t inputFrequency = SystemCoreClock;

    // The timer frequency may be a submultiple of the CPU frequency, due to the
    // bus at which the peripheral is connected being slower.
    // The RCC-ZCFGR register tells us how slower the APB bus is running.
    // The following formula takes into account that if the APB1 clock is
    // divided by a factor of two or grater, the timer is clocked at twice the
    // bus interface.
    // TODO: Check if this and the code are correct
    // TODO: Keep the Cortex M3 support
    if (inputClock == InputClock::APB1)
    {
        if (RCC->CFGR & RCC_CFGR_PPRE1_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> 10) & 0x3);
        }
    }
    else
    {
        if (RCC->CFGR & RCC_CFGR_PPRE2_2)
        {
            inputFrequency /= 1 << ((RCC->CFGR >> 13) >> 0x3);
        }
    }

    return inputFrequency;
}

inline uint32_t TimerUtils::getPrescalerInputFrequency(TIM_TypeDef *timer)
{
    return getPrescalerInputFrequency(getTimerInputClock(timer));
}

inline uint32_t TimerUtils::getFrequency(TIM_TypeDef *timer)
{
    return getPrescalerInputFrequency(timer) / (1 + timer->PSC);
}

inline float TimerUtils::toMicroSeconds(TIM_TypeDef *timer)
{
    return (1.0f * timer->CNT * 1e6 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline uint64_t TimerUtils::toIntMicroSeconds(TIM_TypeDef *timer)
{
    return ((uint64_t)timer->PSC * 1e6 * (uint64_t)(1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
}

inline float TimerUtils::toMilliSeconds(TIM_TypeDef *timer)
{
    return (1.0f * timer->CNT * 1e3 * (1 + timer->PSC)) /
           getPrescalerInputFrequency(timer);
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
