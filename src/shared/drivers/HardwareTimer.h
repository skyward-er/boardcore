/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_DRIVERS_HARDWARETIMER_H
#define SRC_SHARED_DRIVERS_HARDWARETIMER_H

#include <kernel/scheduler/scheduler.h>
#include <miosix.h>
#include <cassert>
#include "Debug.h"

#if !defined _BOARD_STM32F429ZI_SKYWARD_HOMEONE &&  \
    !defined _BOARD_STM32F429ZI_STM32F4DISCOVERY && \
    !defined _BOARD_STM32F429ZI_SKYWARD_ROGALLINA
#error "Unsupported board!"
#endif

using miosix::FastInterruptDisableLock;

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
     * @return unsigned int Prescaler input frequency
     */
    static unsigned int getPrescalerInputFrequency(InputClock input_clock)
    {
        // The global variable SystemCoreClock from ARM's CMSIS allows to
        // know
        // the CPU frequency.
        unsigned int freq = SystemCoreClock;

// The position of the PPRE1 bit in RCC->CFGR is different in some stm32
#ifdef _ARCH_CORTEXM3_STM32
        const unsigned int ppre1 = 8;
#error "Architecture not currently supported"
#else  // stm32f2 and f4
        const unsigned int ppre1 = 10;
        const unsigned int ppre2 = 13;
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

template <typename T, unsigned Tim>
class HardwareTimer
{
    using TimerType = HardwareTimer<T, Tim>;

    static_assert(std::is_same<T, uint32_t>::value ||
                      std::is_same<T, uint16_t>::value,
                  "Timer output type must be either uint16_t or uint32_t");

    static_assert(!((Tim == 2 || Tim == 5) && std::is_same<T, uint16_t>::value),
                  "Tim2 and Tim5 are 32 bit timers!");

public:
    inline static TimerType& instance()
    {
        static TimerType timer;
        return timer;
    }

    inline T start()
    {
        if (!ticking)
        {
            ticking  = true;
            TIM->CNT = 0;  // Reset the counter

            TIM->CR1 |= TIM_CR1_CEN;
            return 0;
        }
        else
        {
            return tick();
        }
    }

    inline T tick() { return TIM->CNT; }

    inline T stop()
    {
        if (ticking)
        {
            T tick = TIM->CNT;
            TIM->CR1 &= ~TIM_CR1_CEN;
            ticking = false;
            return tick;
        }

        return 0;
    }

    bool isTicking() { return ticking; }

    /**
     * @brief Converts from ticks to microseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMicroSeconds(T ticks)
    {
        return (1.0f * ticks * 1000000 * (1 + prescaler)) /
               (float)TimerUtils::getPrescalerInputFrequency(clk);
    }

    /**
     * @brief Converts from ticks to milliseconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toMilliSeconds(T ticks)
    {
        return (1.0f * ticks * 1000 * (1 + prescaler)) /
               (float)TimerUtils::getPrescalerInputFrequency(clk);
    }

    /**
     * @brief Converts from ticks to seconds, using the current prescaler
     * setting
     *
     * @param ticks
     * @return float
     */
    float toSeconds(T ticks)
    {
        return (1.0f * ticks * (1 + prescaler)) /
               (float)TimerUtils::getPrescalerInputFrequency(clk);
    }

    /*static float toSeconds(T tick) {}
    static float toMilliseconds(T tick) {}
    static float toMicroSeconds(T tick) {}*/

    /**
     * @brief Sets the prescaler value.
     * The tick frequency is defined as the clock frequency value of the timer
     * divided by the prescaler. See the datasheet for further information.
     */
    void setPrescaler(uint16_t prescaler)
    {
        // reset();
        TIM->PSC = prescaler;
        TIM->EGR =
            TIM_EGR_UG;  // Send an update event to load the new prescaler
        // value
        this->prescaler = prescaler;
    }

    /**
     * @brief Set the auto reload value.
     * The auto reload value is the maximum value the timer can count to.
     * After this value is reached, the timers counts back from 0.
     */
    void setAutoReload(T auto_reload)
    {
        if (ticking)
        {
            stop();
        }
        this->auto_reload = auto_reload;
        TIM->ARR          = auto_reload;

        TIM->EGR =
            TIM_EGR_UG;  // Send an update event to load the new auto-reload
    }

private:
    HardwareTimer()
    {
        switch (Tim)
        {
            case 1:
                TIM_EN = RCC_APB2ENR_TIM1EN;
                TIM    = TIM1;
                break;
            default:  // Use TIM2 as default
                TRACE("Wrong timer selected. Using TIM2.\n");
            case 2:
                TIM_EN = RCC_APB1ENR_TIM2EN;
                TIM    = TIM2;
                break;
            case 3:
                TIM_EN = RCC_APB1ENR_TIM3EN;
                TIM    = TIM3;
                break;
            case 4:
                TIM_EN = RCC_APB1ENR_TIM4EN;
                TIM    = TIM4;
                break;
            case 5:
                TIM_EN = RCC_APB1ENR_TIM5EN;
                TIM    = TIM5;
                break;
            case 6:
                TIM_EN = RCC_APB1ENR_TIM6EN;
                TIM    = TIM6;
                break;
            case 7:
                TIM_EN = RCC_APB1ENR_TIM7EN;
                TIM    = TIM7;
                break;
            case 8:
                TIM_EN = RCC_APB2ENR_TIM8EN;
                TIM    = TIM8;
                break;
            case 9:
                TIM_EN = RCC_APB2ENR_TIM9EN;
                TIM    = TIM9;
                break;
            case 10:
                TIM_EN = RCC_APB2ENR_TIM10EN;
                TIM    = TIM10;
                break;
            case 11:
                TIM_EN = RCC_APB2ENR_TIM11EN;
                TIM    = TIM11;
                break;
            case 12:
                TIM_EN = RCC_APB1ENR_TIM12EN;
                TIM    = TIM12;
                break;
            case 13:
                TIM_EN = RCC_APB1ENR_TIM13EN;
                TIM    = TIM13;
                break;
            case 14:
                TIM_EN = RCC_APB1ENR_TIM14EN;
                TIM    = TIM14;
                break;
        }

        if (Tim == 1 || (Tim >= 8 && Tim <= 11))
        {
            clk = TimerUtils::InputClock::APB2;

            FastInterruptDisableLock dLock;
            RCC->APB2ENR |= TIM_EN;
            RCC_SYNC();
        }
        else
        {
            clk = TimerUtils::InputClock::APB1;

            FastInterruptDisableLock dLock;
            RCC->APB1ENR |= TIM_EN;
            RCC_SYNC();
        }

        // Reset control registers
        TIM->CR1 = 0;
        TIM->CR2 = 0;

        auto_reload = static_cast<T>(-1);  // Max value of T (unsigned int)
        TIM->ARR    = auto_reload;

        prescaler = 0;
        TIM->PSC  = prescaler;

        TIM->EGR =
            TIM_EGR_UG;  // Send an update event to load the new prescaler and
        // auto reload values
    }

    TIM_TypeDef* TIM;
    uint32_t TIM_EN;
    TimerUtils::InputClock clk;

    T auto_reload;
    uint16_t prescaler;

    bool ticking = false;
};

#endif /* SRC_SHARED_DRIVERS_HARDWARETIMER_H */
