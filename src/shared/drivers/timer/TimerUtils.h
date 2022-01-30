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
 * @brief Trigger sourcer modes.
 *
 * Here a quick recap of the internal trigger sources:
 *
 *       ITR0  ITR1  ITR2  ITR3
 * TIM1  TIM5  TIM2  TIM3  TIM4
 * TIM2  TIM1  TIM8  TIM3  TIM4
 * TIM3  TIM1  TIM2  TIM5  TIM4
 * TIM4  TIM1  TIM2  TIM3  TIM8
 * TIM5  TIM2  TIM3  TIM4  TIM8
 * TIM6  ----  ----  ----  ----
 * TIM7  ----  ----  ----  ----
 * TIM8  TIM1  TIM2  TIM4  TIM5
 * TIM9  TIM2  TIM3  TIM10 TIM11
 * TIM10 ----  ----  ----  ----
 * TIM11 ----  ----  ----  ----
 * TIM12 TIM4  TIM5  TIM13 TIM14
 * TIM13 ----  ----  ----  ----
 * TIM14 ----  ----  ----  ----
 */
enum class TriggerSource : uint16_t
{
    /**
     * @brief Internal trigger 0.
     */
    ITR0 = 0,

    /**
     * @brief Internal trigger 1.
     */
    ITR1 = TIM_SMCR_TS_0,

    /**
     * @brief Internal trigger 2.
     */
    ITR2 = TIM_SMCR_TS_1,

    /**
     * @brief Internal trigger 3.
     */
    ITR3 = TIM_SMCR_TS_1 | TIM_SMCR_TS_0,

    /**
     * @brief TI1 edge detector.
     */
    TI1F_ED = TIM_SMCR_TS_2,

    /**
     * @brief Filtered timer input 1.
     */
    TI1FP1 = TIM_SMCR_TS_2 | TIM_SMCR_TS_0,

    /**
     * @brief Filtered timer input 2.
     */
    TI2FP2 = TIM_SMCR_TS_2 | TIM_SMCR_TS_1
};

enum class MasterMode : uint32_t
{
    /**
     * @brief Only the updateGeneration() function is used as trigger
     * output.
     */
    RESET = 0,

    /**
     * @brief Only the timer enable is used as trigger output.
     *
     * This is useful to start several timers at the same time.
     */
    ENABLE = TIM_CR2_MMS_0,

    /**
     * @brief The UEV is selected as trigger output.
     *
     * This is useful when one timer is used as a prescaler for another
     * timer.
     */
    UPDATE = TIM_CR2_MMS_1,

    /**
     * @brief The trigger output send a positive pulse when the OC1IF flag
     * is to be set (even if it was already high), as soon as a capture or a
     * compare match occurred.
     */
    COMPARE_PULSE = TIM_CR2_MMS_1 | TIM_CR2_MMS_0,

    /**
     * @brief OC1REF signal is used as trigger output (TRGO).
     */
    OC1REF_OUTPUT = TIM_CR2_MMS_2,

    /**
     * @brief OC2REF signal is used as trigger output (TRGO).
     */
    OC2REF_OUTPUT = TIM_CR2_MMS_2 | TIM_CR2_MMS_0,

    /**
     * @brief OC3REF signal is used as trigger output (TRGO).
     */
    OC3REF_OUTPUT = TIM_CR2_MMS_2 | TIM_CR2_MMS_1,

    /**
     * @brief OC4REF signal is used as trigger output (TRGO).
     */
    OC4REF_OUTPUT = TIM_CR2_MMS
};

enum class SlaveMode : uint16_t
{
    /**
     * @brief Slave mode disabled.
     *
     * The clock is only enabled by software.
     */
    DISABLED = 0,

    /**
     * @brief Reset mode.
     *
     * Rising edge of the selected trigger input (TRGI) reinitialize the
     * counter and generates an update of the registers.
     */
    RESET_MODE = TIM_SMCR_SMS_2,

    /**
     * @brief Gated mode.
     *
     * The counter clock is eneabled when the trigger input (TRGI) is high.
     * The counter stops (but is not reset) as soon as the trigger becomes
     * low. Counter starts and stops are both controlled.
     *
     * Note: The gated mode must not be used if TI1F_ED is selected as the
     * trigger input.
     */
    GATED_MODE = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_0,

    /**
     * @brief Trigger mode.
     *
     * The counter starts on a rising edge of the trigger TRGI (but it is
     * not reset). Only the start of the counter is controlled.
     */
    TRIGGER_MODE = TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1,

    /**
     * @brief External clock mode 1.
     *
     * Rising edges of the selected trigger (TRGI) clock the counter
     */
    EXTERNAL_CLOCK_MODE_1 = TIM_SMCR_SMS
};

enum class OutputCompareMode : uint16_t
{
    /**
     * @brief The comparison between the output compare register and the
     * counter has no effect on the outputs.
     */
    FROZEN = 0,

    /**
     * @brief Set channel to active level on match.
     */
    ACTIVE_ON_MATCH = 0x1,

    /**
     * @brief Set channel to inactive level on match.
     */
    INACTIVE_ON_MATCH = 0x2,

    /**
     * @brief The output toggles when the output compare register and the
     * counter match.
     */
    TOGGLE = 0x3,

    /**
     * @brief Output is forced low
     */
    FORCE_INACTIVE = 0x4,

    /**
     * @brief Output is forced high
     */
    FORCE_ACTIVE = 0x5,

    /**
     * @brief Output is active as long as the counter is smaller than the
     * compare register (reverse when downcounting).
     */
    PWM_MODE_1 = 0x6,

    /**
     * @brief Output is active as long as the counter is greater than the
     * compare register (reverse when downcounting).
     */
    PWM_MODE_2 = 0x7
};

enum class OutputComparePolarity : uint16_t
{
    ACTIVE_HIGH = 0,
    ACTIVE_LOW  = 0x1
};

enum class Channel : int
{
    CHANNEL_1 = 0,
    CHANNEL_2 = 1,
    CHANNEL_3 = 2,
    CHANNEL_4 = 3
};

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
 * If the target frequency is above the prescaler input frequency, the returned
 * value will be 0 which is the maximum.
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
    int32_t targetPrescaler =
        TimerUtils::getPrescalerInputFrequency(timer) / targetFrequency - 1;
    return targetPrescaler >= 0 ? targetPrescaler : 0;
}

}  // namespace Boardcore
