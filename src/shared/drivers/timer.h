/* Copyright (c) 2017-2018 Skyward Experimental Rocketry
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

#ifndef SRC_SHARED_DRIVERS_TIMER_H
#define SRC_SHARED_DRIVERS_TIMER_H

#include <Singleton.h>
#include <assert.h>
#include "Common.h"

template <typename T, unsigned Tim>
class Timer : Singleton<Timer<T, Tim>>
{
    typedef Singleton<Timer<T, Tim>> SingletonType;
    friend class Singleton<Timer<T, Tim>>;

    // Check if template arguments are consistent.

    static_assert(std::is_same<T, uint32_t>::value ||
                      std::is_same<T, uint16_t>::value,
                  "Timer output type must be either uint16_t or uint32_t");

    static_assert(
        (Tim >= 2 && Tim <= 5) || (Tim >= 12 && Tim <= 14),
        "Unsupported timer. Supported timers are: Tim 2,3,4,5,12,13,14.");

    static_assert(!((Tim == 2 || Tim == 5) && std::is_same<T, uint16_t>::value),
                  "Tim2 and Tim5 are 32 bit timers!");

public:
    inline static T start() { return SingletonType::getInstance()->_start(); }

    static bool isTicking()
    {
        return SingletonType::getInstance()->_isTicking();
    }

    inline static T stop() { return SingletonType::getInstance()->_stop(); }

    static void reset() { SingletonType::getInstance()->_reset(); }

    inline static T tick() { return SingletonType::getInstance()->_tick(); }

    static T clockFrequency()
    {
        return SingletonType::getInstance()->clock_frequency_;
    }

    /**
     * @brief Sets the prescaler of the timer to the desired value, or to the
     * default
     * one if no parameter is passed. This action resets the timer.
     *
     * @param prescaler
     */
    static void setPrescaler(uint16_t prescaler = 0)
    {
        SingletonType::getInstance()->_setPrescaler(prescaler);
    }

    static void setAutoReload(T auto_reload = 0xFFFFFFFF)
    {
        SingletonType::getInstance()->_setAutoReload(auto_reload);
    }

    static float microSeconds(T ticks)
    {
        return (float)ticks / clockFrequency() * 1000000;
    }

    static float milliSeconds(T ticks)
    {
        return (float)ticks / (float)clockFrequency() * 1000;
    }

    static float seconds(T ticks)
    {
        return (float)ticks / (float)clockFrequency();
    }

private:
    Timer()
    {
        switch (Tim)
        {
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

            default:  // Use timer2 as default timer
                TIM_EN = RCC_APB1ENR_TIM2EN;
                TIM    = TIM2;
        }
        clock_frequency_ = 60000000;
        prescaler_       = 0;
        _init();
    }

    void _init(uint16_t prescaler = 0, T auto_reload = 0xFFFFFFFF)
    {
        // Reset control registers
        TIM->CR1 = TIM_CR1_OPM;
        TIM->CR2 = 0;

        TIM->ARR = auto_reload;
        TIM->PSC = prescaler;

        TIM->EGR = 0x01;  // Send an update event to load the new prescaler and
                          // auto reload values
    }
    /**
     *
     * @brief Starts the timer
     */
    inline T _start()
    {
        if (!ticking_)
        {
            ticking_ = true;
            RCC->APB1ENR |= TIM_EN;
            TIM->CNT = 0;  // Reset the counter

            TIM->CR1 |= TIM_CR1_CEN;
            return 0;
        }
        else
        {
            return _tick();
        }
    }

    inline T _tick() { return TIM->CNT; }

    /**
     * @brief Stops the timer
     */
    inline T _stop()
    {
        if (ticking_)
        {
            T tick = TIM->CNT;
            RCC->APB1ENR &= ~TIM_EN;
            TIM->CR1 &= ~TIM_CR1_CEN;
            ticking_ = false;
            return tick;
        }

        return 0;
    }

    void _setAutoReload(T auto_reload = 0xFFFFFFFF)
    {
        if (ticking_)
        {
            _stop();
        }
        TIM->ARR = auto_reload;
    }

    /**
     * @brief Stops the timer and sets the counter to 0.
     */
    void _reset()
    {
        if (ticking_)
        {
            _stop();
        }

        TIM->CNT = 0;  // Reset the counter
    }

    bool _isTicking() { return ticking_; }

    /**
     * @brief Sets the prescaler of the timer to the desired value, or to the
     * default
     * one if no parameter is passed. This action resets the timer.
     *
     * @param prescaler
     */
    void _setPrescaler(uint16_t prescaler = 0)
    {
        _reset();
        TIM->PSC = prescaler;
        TIM->EGR = 0x01;  // Send an update event to load the new prescaler
                          // value
    }

    TIM_TypeDef* TIM;
    uint32_t TIM_EN;

    uint32_t clock_frequency_;
    uint16_t prescaler_;

    bool ticking_ = false;
};

/*template <unsigned TIM>
using Timer32 = Timer<uint32_t, TIM>;

template <unsigned TIM>
using Timer16 = Timer<uint16_t, TIM>;*/

#endif /* SRC_SHARED_DRIVERS_TIMER_H */
