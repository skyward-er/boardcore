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

#include <interfaces/arch_registers.h>

#include "TimerUtils.h"

namespace Boardcore
{

/**
 * @brief Driver for STM32 basic timers.
 *
 * This driver applies to the whole STM32F4xx family.
 *
 * Basic timers main features are:
 * - 16bit auto-reload upcounter
 * - 16bit programmable prescaler used to divide (also "on the fly") the counter
 * clock frequency by any factor between 1 and 65536
 * - Interrupt/DMA generation on the update event
 *
 * TIM6 and TIM7 are basic timers.
 *
 * You can use any other timer as a basic timer. However TIM9 to TIM 14 can't
 * generate DMA requests.
 *
 * The main block of the programmable timer is a 16-bit upcounter which is
 * incremented every clock cycle, the clock frequency can be divided by a 16-bit
 * prescaler and the counter resets when it reaches the auto-reload value.
 *
 * Every time the counter reaches the auto-reload value, an UPDATE EVENT (UEV)
 * is fired and the counter is reset to 0. You can also generate the UEV by
 * software.
 *
 * The counter is working only when the prescaler is active, thus receiving the
 * clock signal.
 *
 * The auto-reload register can be preloaded, meaning that there you can use a
 * preload register which acts as a buffer. If enabled, when you change the
 * auto-reload register, its content is transfered into the shadow register
 * (they became active) at each UEV, otherwise the new value takes effect
 * immidiately.
 *
 * You can change on the fly the prescaler value as well as the auto-reload
 * register and the counter.
 *
 * When the UEV occurs, all the registers are updated and the update flag is
 * set:
 * - The buffer of the prescaler is reloaded with the preload value (PSC)
 * - The auto-reload shadow register is updated with the preload value (ARR)
 *
 * The clock source is provided by the internal clock source. Specifically:
 * - APB1: TIM2-7/12-15
 * - APB2: TIM1/8-11
 */
class BasicTimer
{
public:
    enum class MasterMode : uint16_t
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
        UPDATE = TIM_CR2_MMS_1
    };

    /**
     * @brief Create a BasicTimer object. Note that this does not resets the
     * timer configuration.
     */
    explicit BasicTimer(TIM_TypeDef *timer);

    TIM_TypeDef *getTimer();

    /**
     * @brief Resets the timer configuration to the default state.
     *
     * This means that:
     * - Auto reload register is not buffered, thus when you modify its value it
     * is taken into effect immediately
     * - One pulse mode disabled
     * - UEV and UG can trigger interrupt and DMA
     * - UEV generation (UG) is enabled
     * - The counter is disabled
     * - Master mode reset
     * - Interrupt and DMA request generation disabled
     * - Counter and prescaler set to 0
     * - Auto reload register set to 65535 (2^16-1)
     */
    void reset();

    void enable();

    void disable();

    bool isEnabled();

    /**
     * @brief The auto reload register is buffered.
     *
     * This means that the shadow register is used and the auto reload value
     * will became active at the next UEV.
     */
    void enableAutoReloadPreload();

    /**
     * @brief Tha auto reload register is not buffered.
     *
     * This means that when you change the auto-reload register, its value is
     * taken into account immediately.
     */
    void disableAutoReloadPreload();

    /**
     * @brief When enabled, the UEV is generated by the counter overflow and
     * software UEV generation.
     */
    void enableUpdateEventGeneration();

    /**
     * @brief The UEV is disabled.
     *
     * However the generateUpdate() function still works.
     */
    void disableUpdateEventGeneration();

    /**
     * @brief Re-initializes the timer counter and generate an update of the
     * registers (the prescaler is cleared too).
     */
    void generateUpdate();

    uint16_t readCounter();

    void setCounter(uint16_t counterValue);

    uint16_t readPrescaler();

    /**
     * @brief Updated the prescaler value.
     *
     * Keep in mind that the new prescaler value is taken into account only at
     * the next update event. If you need to change it immediately you need to
     * call generateUpdate() and make sure that UEV generation is enabled (which
     * is by default).
     */
    void setPrescaler(uint16_t prescalerValue);

    int getFrequency();

    /**
     * @brief Allows to set directly the frequency of the timer's clock.
     *
     * @param frequency Target frequency for the timer's clock.
     */
    void setFrequency(int frequency);

    uint16_t readAutoReloadRegister();

    void setAutoReloadRegister(uint16_t autoReloadValue);

    void enableUpdateInterrupt();

    void disableUpdateInterrupt();

    void enableUpdateDMARequest();

    void disableUpdateDMARequest();

    void enableOnePulseMode();

    void enableUGInterruptAndDMA();

    void disableUGInterruptAndDMA();

    void setMasterMode(MasterMode masterMode);

    static void clearUpdateInterruptFlag(TIM_TypeDef *timer);

protected:
    TIM_TypeDef *timer;
};

inline BasicTimer::BasicTimer(TIM_TypeDef *timer) : timer(timer) {}

inline TIM_TypeDef *BasicTimer::getTimer() { return timer; }

inline void BasicTimer::reset()
{
    timer->CR1  = 0;
    timer->CR2  = 0;
    timer->DIER = 0;
    timer->CNT  = 0;
    timer->PSC  = 0;
    timer->ARR  = 0xFFFF;
}

inline void BasicTimer::enable() { timer->CR1 |= TIM_CR1_CEN; }

inline void BasicTimer::disable() { timer->CR1 &= ~TIM_CR1_CEN; }

inline bool BasicTimer::isEnabled() { return timer->CR1 & TIM_CR1_CEN; }

inline void BasicTimer::enableAutoReloadPreload()
{
    timer->CR1 |= TIM_CR1_ARPE;
}

inline void BasicTimer::disableAutoReloadPreload()
{
    timer->CR1 &= ~TIM_CR1_ARPE;
}

inline void BasicTimer::enableUpdateEventGeneration()
{
    timer->CR1 &= ~TIM_CR1_UDIS;
}

inline void BasicTimer::disableUpdateEventGeneration()
{
    timer->CR1 |= TIM_CR1_UDIS;
}

inline void BasicTimer::generateUpdate() { timer->EGR |= TIM_EGR_UG; }

inline uint16_t BasicTimer::readCounter() { return timer->CNT; }

inline void BasicTimer::setCounter(uint16_t counterValue)
{
    timer->CNT = counterValue;
}

inline uint16_t BasicTimer::readPrescaler() { return timer->PSC; }

inline void BasicTimer::setPrescaler(uint16_t prescalerValue)
{
    timer->PSC = prescalerValue;
}

inline int BasicTimer::getFrequency()
{
    return TimerUtils::getFrequency(timer);
}

inline void BasicTimer::setFrequency(int frequency)
{
    setPrescaler(TimerUtils::computePrescalerValue(timer, frequency));
}

inline uint16_t BasicTimer::readAutoReloadRegister() { return timer->ARR; }

inline void BasicTimer::setAutoReloadRegister(uint16_t autoReloadValue)
{
    timer->ARR = autoReloadValue;
}

inline void BasicTimer::enableUpdateInterrupt() { timer->DIER |= TIM_DIER_UIE; }

inline void BasicTimer::disableUpdateInterrupt()
{
    timer->DIER &= ~TIM_DIER_UIE;
}

inline void BasicTimer::enableUpdateDMARequest()
{
    timer->DIER |= TIM_DIER_UDE;
}

inline void BasicTimer::disableUpdateDMARequest()
{
    timer->DIER &= ~TIM_DIER_UDE;
}

inline void BasicTimer::enableOnePulseMode() { timer->CR1 |= TIM_CR1_OPM; }

inline void BasicTimer::enableUGInterruptAndDMA()
{
    timer->CR1 &= ~TIM_CR1_URS;
}

inline void BasicTimer::disableUGInterruptAndDMA()
{
    timer->CR1 |= TIM_CR1_URS;
}

inline void BasicTimer::setMasterMode(MasterMode masterMode)
{
    // First clear the configuration
    timer->CR2 &= ~TIM_CR2_MMS;

    // Set the new value
    timer->CR2 |= static_cast<uint32_t>(masterMode);
}

inline void BasicTimer::clearUpdateInterruptFlag(TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_UIF;
}

}  // namespace Boardcore
