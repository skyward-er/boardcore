/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Davide Mor
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

#include <miosix.h>

#include <type_traits>

namespace Boardcore
{

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
        const uint32_t ppre2 = 11;
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

    template <typename Type>
    static float toMicroSeconds(Type &tim, typename Type::Output ticks)
    {
        return (1.0f * ticks * 1000000 * (1 + tim.prescaler)) /
               tim.prescaler_freq;
    }

    template <typename Type>
    static uint64_t toIntMicroSeconds(Type &tim, typename Type::Output ticks)
    {
        return ((uint64_t)ticks * 1000000 * (uint64_t)(1 + tim.prescaler)) /
               (uint64_t)tim.prescaler_freq;
    }

    template <typename Type>
    static float toMilliSeconds(Type &tim, typename Type::Output ticks)
    {
        return (1.0f * ticks * 1000 * (1 + tim.prescaler)) / tim.prescaler_freq;
    }

    template <typename Type>
    static float toSeconds(Type &tim, typename Type::Output ticks)
    {
        return (1.0f * ticks * (1 + tim.prescaler)) / tim.prescaler_freq;
    }

    template <typename Type>
    static float getResolution(Type &tim)
    {
        // Resolution in us = number microseconds in one tick
        return toMicroSeconds(tim, 1);
    }

    template <typename Type>
    static float getMaxDuration(Type &tim)
    {
        // Maximum duration = number of seconds to count to the auto_reload
        // (reset) value.
        return toSeconds(tim, tim.auto_reload);
    }
};

/**
 * @brief Selects to operating mode of the timer.
 */
enum class TimerMode
{
    Single,  //< Use a single timer as a source of time.
    Chain    //< Chain two timers as a source of time.
};

/**
 * @brief Which trigger connection to use.
 *
 * This dictates the trigger for the slave timer,
 * this must be choosen depending on the master slave.
 *
 * Here you can see a table with all the possible combinations
 * taken directly from the STM32 datasheet, on the top row you can
 * see the trigger pin, while on the left there is the choosen slave timer,
 * by looking at the intersection you can find the connected master timer.
 *
 * PS: The datasheet where this comes from is for:
 * - STM32F101xx
 * - STM32F102xx
 * - STM32F103xx
 * - STM32F105xx
 * - STM32F107xx
 *
 * ```
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
 * ```
 *
 */
enum class TimerTrigger
{
    ITR0 = 0,
    ITR1 = TIM_SMCR_TS_0,
    ITR2 = TIM_SMCR_TS_1,
    ITR3 = TIM_SMCR_TS_0 | TIM_SMCR_TS_1
};

/**
 * @brief Class for handling Hardware Timers and perform time unit conversions
 *
 * @tparam Type type to use to access timer register (uint32_t if 32 bit timer,
 * uint16_t if 16 bit timer)
 * @tparam Operating mode of the timer (see TimerMode)
 */
template <typename Type, TimerMode Mode = TimerMode::Single>
class HardwareTimer;

template <typename Type>
class HardwareTimer<Type, TimerMode::Single>
{
    friend class TimerUtils;

    static_assert(std::is_same<Type, uint32_t>::value ||
                      std::is_same<Type, uint16_t>::value,
                  "Type must be either uint32_t or uint16_t.");

public:
    using Output = Type;

    /**
     * @brief Creates a new HardwareTimer instance
     *
     * @param    timer     The timer to use (pointer to timer registers struct)
     * @param    psc_input_freq Input frequency of the timer's prescaler, see
     *                          TimerUtils::getPrescalerInputFrequency()
     */
    HardwareTimer(TIM_TypeDef *timer, uint32_t psc_input_freq)
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

    /**
     * @brief Converts from ticks to microseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return uint64_t
     */
    uint64_t toIntMicroSeconds(Type ticks)
    {
        return TimerUtils::toIntMicroSeconds(*this, ticks);
    }

    /**
     * @brief Converts from ticks to microseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMicroSeconds(Type ticks)
    {
        return TimerUtils::toMicroSeconds(*this, ticks);
    }

    /**
     * @brief Converts from ticks to milliseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMilliSeconds(Type ticks)
    {
        return TimerUtils::toMilliSeconds(*this, ticks);
    }

    /**
     * @brief Converts from ticks to seconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toSeconds(Type ticks) { return TimerUtils::toSeconds(*this, ticks); }

    /**
     * @brief Returns the current resolution of the timer in microseconds.
     * @return Resolution in microseconds
     */
    float getResolution() { return TimerUtils::getResolution(*this); }

    /**
     * @brief Returns the maximum time the timer can measure before restarting
     * from 0, in seconds.
     *
     * @return Maximum timer duration in seconds.
     */
    float getMaxDuration() { return TimerUtils::getMaxDuration(*this); }

private:
    TIM_TypeDef *tim;
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
inline Type HardwareTimer<Type>::tick()
{
    return tim->CNT;
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
class HardwareTimer<Type, TimerMode::Chain>
{
    friend class TimerUtils;

    // TODO: Does this still work with two 32bit timers?
    static_assert(std::is_same<Type, uint32_t>::value,
                  "Type must be uint32_t.");

public:
    using Output = Type;

    /**
     * @brief Creates a new HardwareTimer instance
     *
     * @param tim The timer to use as master (pointer to timer registers
     * struct)
     * @param slave The timer to use as slave (pointer to timer registers
     * struct)
     * @param trigger The slave input trigger, should be choosen according to
     * master and slave (see TimerTrigger)
     * @param prescaler_freq Input frequency of the timer's prescaler, see
     * TimerUtils::getPrescalerInputFrequency()
     */
    HardwareTimer(TIM_TypeDef *tim, TIM_TypeDef *slave, TimerTrigger trigger,
                  uint32_t prescaler_freq);

    Type start();

    Type stop();

    Type tick();

    void setPrescaler(uint16_t prescaler);

    void setAutoReload(Type auto_reload);

    uint64_t toIntMicroSeconds(Type ticks)
    {
        return TimerUtils::toIntMicroSeconds(*this, ticks);
    }

    float toMicroSeconds(Type ticks)
    {
        return TimerUtils::toMicroSeconds(*this, ticks);
    }

    float toMilliSeconds(Type ticks)
    {
        return TimerUtils::toMilliSeconds(*this, ticks);
    }

    float toSeconds(Type ticks) { return TimerUtils::toSeconds(*this, ticks); }

    float getResolution() { return toMicroSeconds(1); }

    float getMaxDuration() { return toSeconds(auto_reload); }

private:
    TIM_TypeDef *tim, *slave;
    uint32_t prescaler_freq;

    bool ticking       = false;
    uint16_t prescaler = 0;
    Type auto_reload =
        static_cast<Type>(-1);  // Max value of Type (Type is unsigned)
};

template <typename Type>
inline HardwareTimer<Type, TimerMode::Chain>::HardwareTimer(
    TIM_TypeDef *tim, TIM_TypeDef *slave, TimerTrigger trigger,
    uint32_t prescaler_freq)
    : tim(tim), slave(slave), prescaler_freq(prescaler_freq)
{
    // Trigger selection from parameter
    const uint8_t ts = static_cast<uint8_t>(trigger);
    // Slave mode selection 0b111 -> External Clock mode 1
    const uint8_t sms = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1 | TIM_SMCR_SMS_2;
    // Master mode selection 0b010 -> Update
    const uint8_t mms = TIM_CR2_MMS_1;

    // Configure timer1 as master and timer2 as slave
    tim->CR2 &= ~TIM_CR2_MMS;
    tim->CR2 |= mms;

    slave->SMCR &= ~(TIM_SMCR_TS | TIM_SMCR_SMS);
    slave->SMCR |= ts | sms;
}

template <typename Type>
inline Type HardwareTimer<Type, TimerMode::Chain>::start()
{
    if (!ticking)
    {
        ticking = true;

        slave->CNT = tim->CNT = 0;

        slave->CR1 |= TIM_CR1_CEN;
        tim->CR1 |= TIM_CR1_CEN;
        return 0;
    }
    else
    {
        return tick();
    }
}

template <typename Type>
inline Type HardwareTimer<Type, TimerMode::Chain>::stop()
{
    if (ticking)
    {
        slave->CR1 &= ~TIM_CR1_CEN;
        tim->CR1 &= ~TIM_CR1_CEN;

        ticking = false;
        return tick();
    }
    else
    {
        return 0;
    }
}

template <typename Type>
inline Type HardwareTimer<Type, TimerMode::Chain>::tick()
{
    return (Type)tim->CNT | ((Type)slave->CNT << (sizeof(Type) * 8 / 2));
}

template <typename Type>
void HardwareTimer<Type, TimerMode::Chain>::setPrescaler(uint16_t prescaler)
{
    this->prescaler = prescaler;
    tim->PSC        = prescaler;
    tim->EGR = TIM_EGR_UG;  // Send an update event to load the new prescaler
                            // value
}

template <typename Type>
void HardwareTimer<Type, TimerMode::Chain>::setAutoReload(Type auto_reload)
{
    this->auto_reload = auto_reload;
    // We only care about the higher bits
    slave->ARR = auto_reload >> (sizeof(Type) * 8 / 2);
    slave->EGR = TIM_EGR_UG;  // Send an update event to load the new
                              // auto-reload
}

}  // namespace Boardcore
