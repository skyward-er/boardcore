/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
#include <miosix.h>

/**
 * @brief Driver for STM32 32bit general purpose timers.
 *
 * This driver adapts the GeneralPurposeTimer class for 32bit timers.
 */
class GeneralPurpose32BitTimer : public GeneralPurposeTimer
{
public:
    /**
     * @brief Create a GeneralPurpose32BitTimer object. Note that this does not
     * resets the timer configuration.
     */
    GeneralPurpose32BitTimer(TIM_TypeDef* timer);

    /**
     * @brief Reads the counter value.
     *
     * @return Counter value
     */
    uint32_t readCounter();

    /**
     * @brief Sets the timer counter value.
     *
     * @param counterValue Counter value to set.
     */
    void setCounter(uint32_t counterValue);

    /**
     * @brief Reads the timer auto-reload register.
     *
     * @return Tiemr auto-reload register value.
     */
    uint32_t readAutoReloadRegister();

    /**
     * @brief Changes the auto-reload register.
     *
     * @param autoReloadValue New auto-reload register value.
     */
    void setAutoReloadRegister(uint32_t autoReloadValue);

    /**
     * @brief Sets the timer capture/compare 1 value.
     */
    void setCaptureCompare1Register(uint32_t value);

    /**
     * @brief Sets the timer capture/compare 2 value.
     */
    void setCaptureCompare2Register(uint32_t value);

    /**
     * @brief Sets the timer capture/compare 3 value.
     */
    void setCaptureCompare3Register(uint32_t value);

    /**
     * @brief Sets the timer capture/compare 4 value.
     */
    void setCaptureCompare4Register(uint32_t value);
};

inline GeneralPurpose32BitTimer::GeneralPurpose32BitTimer(TIM_TypeDef* timer)
    : GeneralPurposeTimer(timer)
{
}

inline uint32_t GeneralPurpose32BitTimer::readCounter() { return timer->CNT; }

inline void GeneralPurpose32BitTimer::setCounter(uint32_t counterValue)
{
    timer->CNT = counterValue;
}

inline uint32_t GeneralPurpose32BitTimer::readAutoReloadRegister()
{
    return timer->ARR;
}

inline void GeneralPurpose32BitTimer::setAutoReloadRegister(
    uint32_t autoReloadValue)
{
    timer->ARR = autoReloadValue;
}

inline void GeneralPurpose32BitTimer::setCaptureCompare1Register(uint32_t value)
{
    timer->CCR1 = value;
}

inline void GeneralPurpose32BitTimer::setCaptureCompare2Register(uint32_t value)
{
    timer->CCR2 = value;
}

inline void GeneralPurpose32BitTimer::setCaptureCompare3Register(uint32_t value)
{
    timer->CCR3 = value;
}

inline void GeneralPurpose32BitTimer::setCaptureCompare4Register(uint32_t value)
{
    timer->CCR4 = value;
}