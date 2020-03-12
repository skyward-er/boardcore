/**
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#pragma once

#include <miosix.h>

#include <type_traits>

class TimerUtils
{
public:
    enum class InputClock
    {
        APB1,
        APB2
    };

    /**
     * @brief returns the timer clock frequency before the prescaler.
     * Function borrowed from the SyncronizedServo class in Miosix.
     *
     * @return Prescaler input frequency
     */
    static uint32_t getPrescalerInputFrequency(InputClock input_clock)
    {
        // The global variable SystemCoreClock from ARM's CMSIS allows to
        // know
        // the CPU frequency.
        uint32_t freq = SystemCoreClock;

// The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const uint32_t ppre1 = 8;
#error "Architecture not currently supported"
#else  // stm32f2 and f4
        const uint32_t ppre1 = 10;
        const uint32_t ppre2 = 13;
#endif
        // The timer frequency may however be a submultiple of the CPU
        // frequency,
        // due to the bus at whch the periheral is connected being slower.
        // The
        // RCC->CFGR register tells us how slower the APB1 bus is running.
        // This formula takes into account that if the APB1 clock is divided
        // by a
        // factor of two or greater, the timer is clocked at twice the bus
        // interface. After this, the freq variable contains the frequency
        // in Hz
        // at which the timer prescaler is clocked.
        if (input_clock == InputClock::APB1)
        {
            if (RCC->CFGR & RCC_CFGR_PPRE1_2)
                freq /= 1 << ((RCC->CFGR >> ppre1) & 0x3);
        }
        else
        {
            if (RCC->CFGR & RCC_CFGR_PPRE2_2)
                freq /= 1 << ((RCC->CFGR >> ppre2) & 0x3);
        }

        return freq;
    }
};

/**
 * @brief Class for handling Hardware Timers and perform time unit conversions
 *
 * @tparam Type type to use to access timer register (uint32_t if 32 bit timer,
 * uint16_t if 16 bit timer)
 */
template <typename Type>
class HardwareTimer
{
    static_assert(std::is_same<Type, uint32_t>::value ||
                      std::is_same<Type, uint16_t>::value,
                  "Type must be either uint32_t or uint16_t.");

public:
    /**
     * @brief Creates a new HardwareTimer instance
     *
     * @param    timer     The timer to use (pointer to timer registers struct)
     * @param    psc_input_freq Input frequency of the timer's prescaler, see
     *                          TimerUtils::getPrescalerInputFrequency()
     */
    HardwareTimer(TIM_TypeDef* timer, uint32_t psc_input_freq)
        : tim(timer), prescaler_freq(psc_input_freq)
    {
    }

    /**
     * @brief Starts the timer returns the current tick
     *
     * @return Current tick
     */
    Type start();

    /**
     * @brief Stops the timer if already started
     *
     * @return Current tick or 0 if already stopped
     */
    Type stop();

    /**
     * @brief Returns current tick
     */
    Type tick();

    /**
     * @brief Converts from ticks to microseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMicroSeconds(Type ticks);

    uint64_t toIntMicroSeconds(Type ticks);

    /**
     * @brief Converts from ticks to milliseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMilliSeconds(Type ticks);

    /**
     * @brief Converts from ticks to seconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toSeconds(Type ticks);

    /**
     * @brief Returns the current resolution of the timer in microseconds.
     * @return Resolution in microseconds
     */
    float getResolution();

    /**
     * @brief Returns the maximum time the timer can measure before restarting
     * from 0, in seconds.
     *
     * @return Maximum timer duration in seconds.
     */
    float getMaxDuration();

    /**
     * @brief Sets the prescaler value.
     * The tick frequency is defined as the clock frequency value of the timer
     * divided by the prescaler. See the datasheet for further information.
     */
    void setPrescaler(uint16_t prescaler);

    /**
     * @brief Set the auto reload value.
     * The auto reload value is the maximum value the timer can count to.
     * After this value is reached, the timers counts back from 0.
     */
    void setAutoReload(Type auto_reload);

private:
    TIM_TypeDef* tim;
    uint32_t prescaler_freq;

    bool ticking       = false;
    uint16_t prescaler = 0;
    Type auto_reload =
        static_cast<Type>(-1);  // Max value of Type (Type is unsigned)
};

template <typename Type>
inline Type HardwareTimer<Type>::start()
{
    if (!ticking)
    {
        ticking  = true;
        tim->CNT = 0;  // Reset the counter

        tim->CR1 |= TIM_CR1_CEN;
        return 0;
    }
    else
    {
        return tick();
    }
}

template <typename Type>
inline Type HardwareTimer<Type>::tick()
{
    return tim->CNT;
}

template <typename Type>
inline Type HardwareTimer<Type>::stop()
{
    if (ticking)
    {
        tim->CR1 &= ~TIM_CR1_CEN;
        ticking = false;
        return tim->CNT;
    }

    return 0;
}

template <typename Type>
void HardwareTimer<Type>::setPrescaler(uint16_t prescaler)
{
    this->prescaler = prescaler;
    tim->PSC        = prescaler;
    tim->EGR = TIM_EGR_UG;  // Send an update event to load the new prescaler
                            // value
}

template <typename Type>
void HardwareTimer<Type>::setAutoReload(Type auto_reload)
{
    this->auto_reload = auto_reload;
    tim->ARR          = auto_reload;

    tim->EGR = TIM_EGR_UG;  // Send an update event to load the new auto-reload
}

template <typename Type>
float HardwareTimer<Type>::toMicroSeconds(Type ticks)
{
    return (1.0f * ticks * 1000000 * (1 + prescaler)) / prescaler_freq;
}

template <typename Type>
uint64_t HardwareTimer<Type>::toIntMicroSeconds(Type ticks)
{
    return ((uint64_t)ticks * 1000000 * (uint64_t)(1 + prescaler)) /
           (uint64_t)prescaler_freq;
}

template <typename Type>
float HardwareTimer<Type>::toMilliSeconds(Type ticks)
{
    return (1.0f * ticks * 1000 * (1 + prescaler)) / prescaler_freq;
}

template <typename Type>
float HardwareTimer<Type>::toSeconds(Type ticks)
{
    return (1.0f * ticks * (1 + prescaler)) / prescaler_freq;
}

template <typename Type>
float HardwareTimer<Type>::getResolution()
{
    // Resolution in us = number microseconds in one tick
    return toMicroSeconds(1);
}

template <typename Type>
float HardwareTimer<Type>::getMaxDuration()
{
    // Maximum duration = number of seconds to count to the auto_reload (reset)
    // value.
    return toSeconds(auto_reload);
}