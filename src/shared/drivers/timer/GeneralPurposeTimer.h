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

#include <type_traits>

#include "BasicTimer.h"

namespace Boardcore
{

/**
 * @brief Driver for STM32 general purpose timers.
 *
 * This driver applies to the whole STM32F4xx family.
 *
 * General purpose timers main features are:
 * - 16bit auto-reload upcounter
 * - 16bit programmable prescaler used to divide (also "on the fly") the counter
 * clock frequency by any factor between 1 and 65536
 * - Up to 4 independent channels for:
 *   - Input capture
 *   - Output compare
 *   - PWM genration (edge-aligned mode)
 *   - One-pulse mode output
 * - Synchronization circuit to control the timer with external signals and to
 * interconnect several timers together
 * - Interrupt generation on the following events:
 *   - Update: counter overflow, counter initialization (by software of internal
 * trigger)
 *   - Trigger event (counter start, stop, initialization or count by internal
 * trigger)
 *   - Input capture
 *   - Ouput compare
 *
 * TIM2 to TIM5 and TIM9 to TIM14 are general purpose timers.
 *
 * You can use 32bit general purpose timers and advance timers as general
 * purpose timers.
 *
 * Although general purpose timers are called the same, there are slight
 * differences in features:
 * - TIM9 and TIM12 have only two channels
 * - TIM10/11/13/14 have only one channel, do not offer synchronization circuit
 * and interrupt can't be generated from trigger events
 *
 * For a basic introduction to timers read BasicTimer description.
 * For a more exhaustive explanation and feature list study the reference
 * manual!
 *
 * The counter clock can be provided by the following clock sources:
 *   - Internal clock
 *   - External clock mode 1 (for TIM9 and TIM12): external input pin
 *   - Internal trigger inputs (for TIM9 and TIM12): connecting the trigger
 * output from another timer
 *
 * The internal clock source is the default clock source. For TIM9 and TIM12 the
 * internal clock source is selected when the slave mode controller is disabled.
 *
 * In External clock mode 1 the counter can count at each rising or falling edge
 * on a selected input.
 *
 * Output compare mode function is used to control an output waveform or
 * indicating when a period of timer has elapse. When a match is found between
 * the capture/compare register and the counter, the compare function outputs
 * the value defined by the output compare mode.
 *
 * PWM mode allows the user to generate a PWM signal with a specific frequency
 * and duty cycle.
 *
 * TODO: Gated mode
 *
 * TODO: Timer synchronization
 *
 * Other timer functionalities still needs to be implemented.
 */
template <typename T>
class GeneralPurposeTimer final : public BasicTimer
{
public:
    static_assert(std::is_same<T, uint16_t>::value ||
                      std::is_same<T, uint32_t>::value,
                  "Type must be either uint16_t or uint32_t.");

    /**
     * @brief Create a GeneralPurposeTimer object. Note that this does not
     * resets the timer configuration but automatically enables the timer
     * peripheral clock.
     */
    explicit GeneralPurposeTimer(TIM_TypeDef *timer);

    /**
     * @brief Disables the peripheral clock.
     */
    ~GeneralPurposeTimer();

    void reset() override;

    void enable() override;

    void disable() override;

    T readCounter();

    void setCounter(T counterValue);

    T readAutoReloadRegister();

    void setAutoReloadRegister(T autoReloadValue);

    void setMasterMode(TimerUtils::MasterMode masterMode) override;

    void setSlaveMode(TimerUtils::SlaveMode slaveMode);

    void setTriggerSource(TimerUtils::TriggerSource triggerSource);

    void enableTriggerInterrupt();

    void disableTriggerInterrupt();

    void enableCaptureCompareInterrupt(TimerUtils::Channel channel);

    void disableCaptureCompareInterrupt(TimerUtils::Channel channel);

    void enableCaptureCompareDMARequest(TimerUtils::Channel channel);

    void disableCaptureCompareDMARequest(TimerUtils::Channel channel);

    void generateTrigger();

    void generateCaptureCompareEvent(TimerUtils::Channel channel);

    /**
     * @brief The capture/compare register is buffered.
     *
     * This means that the shadow register is used and the capture/compare
     * value will became active at the next UEV.
     */
    void enableCaptureComparePreload(TimerUtils::Channel channel);

    /**
     * @brief The capture/compare register is not buffered.
     *
     * This means that when you change the capture/compare register, its value
     * is taken into account immediately.
     */
    void disableCaptureComparePreload(TimerUtils::Channel channel);

    void setOutputCompareMode(TimerUtils::Channel channel,
                              TimerUtils::OutputCompareMode modeChannel);

    void enableCaptureCompareOutput(TimerUtils::Channel channel);

    void enableCaptureCompareComplementaryOutput(TimerUtils::Channel channel);

    /**
     * Note that after the output is disabled, its level is then function of
     * MOE, OSSI, OSSR, OIS1, OIS1N and CC1NE bits for timers TIM1 and TIM8.
     *
     * Practically disabled, so low state.
     */
    void disableCaptureCompareOutput(TimerUtils::Channel channel);

    /**
     * Same as non complementary outputs but their idle state is high.
     */
    void disableCaptureCompareComplementaryOutput(TimerUtils::Channel channel);

    bool isCaptureComapreOutputEnabled(TimerUtils::Channel channel);

    bool isCaptureComapreComplementaryOutputEnabled(
        TimerUtils::Channel channel);

    void setCaptureComparePolarity(TimerUtils::Channel channel,
                                   TimerUtils::OutputComparePolarity polarity);

    void setCaptureCompareComplementaryPolarity(
        TimerUtils::Channel channel,
        TimerUtils::OutputComparePolarity polarity);

    void setCaptureCompareRegister(TimerUtils::Channel channel, T value);

    T readCaptureCompareRegister(TimerUtils::Channel channel);

    static void clearTriggerInterruptFlag(TIM_TypeDef *timer);

    static void clearCaptureCompareInterruptFlag(TimerUtils::Channel channel,
                                                 TIM_TypeDef *timer);
};

/**
 * @brief General purpose 16bit timer.
 *
 * If a timer is natively 32bit, it can be used as 16bit (or even as a basic
 * timer).
 */
using GP16bitTimer = GeneralPurposeTimer<uint16_t>;

/**
 * @brief General purpose 32bit timer.
 */
using GP32bitTimer = GeneralPurposeTimer<uint32_t>;

template <typename T>
inline GeneralPurposeTimer<T>::GeneralPurposeTimer(TIM_TypeDef *timer)
    : BasicTimer(timer)
{
}

template <typename T>
inline GeneralPurposeTimer<T>::~GeneralPurposeTimer()
{
    ClockUtils::disablePeripheralClock(timer);
}

template <typename T>
inline void GeneralPurposeTimer<T>::reset()
{
    timer->CR1   = 0;
    timer->CR2   = 0;
    timer->SMCR  = 0;
    timer->DIER  = 0;
    timer->EGR   = 0;
    timer->CCMR1 = 0;
    timer->CCMR2 = 0;
    timer->CCER  = 0;
    timer->CNT   = 0;
    timer->PSC   = 0;
    timer->ARR   = static_cast<uint32_t>(0xFFFFFFFFF);
    timer->CCR1  = 0;
    timer->CCR2  = 0;
    timer->CCR3  = 0;
    timer->CCR4  = 0;
    timer->DCR   = 0;
    timer->DMAR  = 0;
    timer->OR    = 0;
}

template <typename T>
inline void GeneralPurposeTimer<T>::enable()
{
    timer->CR1 |= TIM_CR1_CEN;

    // On TIM1 and TIM8 the outputs are enabled only if the MOE bit in the BDTR
    // register is set
    if (timer == TIM1 || timer == TIM8)
    {
        timer->BDTR |= TIM_BDTR_MOE;
    }
}

template <typename T>
inline void GeneralPurposeTimer<T>::disable()
{
    timer->CR1 &= ~TIM_CR1_CEN;

    // On TIM1 and TIM8 the outputs are enabled only if the MOE bit in the BDTR
    // register is set
    if (timer == TIM1 || timer == TIM8)
    {
        timer->BDTR |= TIM_BDTR_MOE;
    }
}

template <typename T>
inline T GeneralPurposeTimer<T>::readCounter()
{
    return timer->CNT;
}

template <typename T>
inline void GeneralPurposeTimer<T>::setCounter(T counterValue)
{
    timer->CNT = counterValue;
}

template <typename T>
inline T GeneralPurposeTimer<T>::readAutoReloadRegister()
{
    return timer->ARR;
}

template <typename T>
inline void GeneralPurposeTimer<T>::setAutoReloadRegister(T autoReloadValue)
{
    timer->ARR = autoReloadValue;
}

template <typename T>
inline void GeneralPurposeTimer<T>::setMasterMode(
    TimerUtils::MasterMode masterMode)
{
    // First clear the configuration
    timer->CR2 &= ~TIM_CR2_MMS;

    // Set the new value
    timer->CR2 |= static_cast<uint16_t>(masterMode);
}

template <typename T>
inline void GeneralPurposeTimer<T>::setSlaveMode(
    TimerUtils::SlaveMode slaveMode)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_SMS;

    // Set the new value
    timer->SMCR |= static_cast<uint16_t>(slaveMode);
}

template <typename T>
inline void GeneralPurposeTimer<T>::setTriggerSource(
    TimerUtils::TriggerSource triggerSource)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_TS;

    // Set the new value
    timer->SMCR |= static_cast<uint16_t>(triggerSource);
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableTriggerInterrupt()
{
    timer->DIER |= TIM_DIER_TIE;
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableTriggerInterrupt()
{
    timer->DIER &= ~TIM_DIER_TIE;
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableCaptureCompareInterrupt(
    TimerUtils::Channel channel)
{
    timer->DIER |= TIM_DIER_CC1IE << static_cast<int>(channel);
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableCaptureCompareInterrupt(
    TimerUtils::Channel channel)
{
    timer->DIER &= ~(TIM_DIER_CC1IE << static_cast<int>(channel));
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableCaptureCompareDMARequest(
    TimerUtils::Channel channel)
{
    timer->DIER |= TIM_DIER_CC1DE << static_cast<int>(channel);
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableCaptureCompareDMARequest(
    TimerUtils::Channel channel)
{
    timer->DIER &= ~(TIM_DIER_CC1DE << static_cast<int>(channel));
}

template <typename T>
inline void GeneralPurposeTimer<T>::generateTrigger()
{
    timer->EGR |= TIM_EGR_TG;
}

template <typename T>
inline void GeneralPurposeTimer<T>::generateCaptureCompareEvent(
    TimerUtils::Channel channel)
{
    timer->EGR |= TIM_EGR_CC1G << static_cast<int>(channel);
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableCaptureComparePreload(
    TimerUtils::Channel channel)
{
    switch (channel)
    {
        case TimerUtils::Channel::CHANNEL_1:
            timer->CCMR1 |= TIM_CCMR1_OC1PE;
            break;
        case TimerUtils::Channel::CHANNEL_2:
            timer->CCMR1 |= TIM_CCMR1_OC2PE;
            break;
        case TimerUtils::Channel::CHANNEL_3:
            timer->CCMR2 |= TIM_CCMR2_OC3PE;
            break;
        case TimerUtils::Channel::CHANNEL_4:
            timer->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
    }
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableCaptureComparePreload(
    TimerUtils::Channel channel)
{
    switch (channel)
    {
        case TimerUtils::Channel::CHANNEL_1:
            timer->CCMR1 &= ~TIM_CCMR1_OC1PE;
            break;
        case TimerUtils::Channel::CHANNEL_2:
            timer->CCMR1 &= ~TIM_CCMR1_OC2PE;
            break;
        case TimerUtils::Channel::CHANNEL_3:
            timer->CCMR2 &= ~TIM_CCMR2_OC3PE;
            break;
        case TimerUtils::Channel::CHANNEL_4:
            timer->CCMR2 &= ~TIM_CCMR2_OC4PE;
            break;
    }
}

template <typename T>
inline void GeneralPurposeTimer<T>::setOutputCompareMode(
    TimerUtils::Channel channel, TimerUtils::OutputCompareMode mode)
{
    switch (channel)
    {
        case TimerUtils::Channel::CHANNEL_1:
            // First clear the configuration
            timer->CCMR1 &= ~TIM_CCMR1_OC1M;

            // Set the new value
            timer->CCMR1 |= static_cast<uint16_t>(mode) << 4;
            break;
        case TimerUtils::Channel::CHANNEL_2:
            // First clear the configuration
            timer->CCMR1 &= ~TIM_CCMR1_OC2M;

            // Set the new value
            timer->CCMR1 |= static_cast<uint16_t>(mode) << 12;
            break;
        case TimerUtils::Channel::CHANNEL_3:
            // First clear the configuration
            timer->CCMR2 &= ~TIM_CCMR2_OC3M;

            // Set the new value
            timer->CCMR2 |= static_cast<uint16_t>(mode) << 4;
            break;
        case TimerUtils::Channel::CHANNEL_4:
            // First clear the configuration
            timer->CCMR2 &= ~TIM_CCMR2_OC4M;

            // Set the new value
            timer->CCMR2 |= static_cast<uint16_t>(mode) << 12;
            break;
    }
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableCaptureCompareOutput(
    TimerUtils::Channel channel)
{
    timer->CCER |= TIM_CCER_CC1E << (static_cast<int>(channel) * 4);
}

template <typename T>
inline void GeneralPurposeTimer<T>::enableCaptureCompareComplementaryOutput(
    TimerUtils::Channel channel)
{
    timer->CCER |= TIM_CCER_CC1NE << (static_cast<int>(channel) * 4);
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableCaptureCompareOutput(
    TimerUtils::Channel channel)
{
    timer->CCER &= ~(TIM_CCER_CC1E << (static_cast<int>(channel) * 4));
}

template <typename T>
inline void GeneralPurposeTimer<T>::disableCaptureCompareComplementaryOutput(
    TimerUtils::Channel channel)
{
    timer->CCER &= ~(TIM_CCER_CC1NE << (static_cast<int>(channel) * 4));
}

template <typename T>
inline bool GeneralPurposeTimer<T>::isCaptureComapreOutputEnabled(
    TimerUtils::Channel channel)
{
    return timer->CCER & (TIM_CCER_CC1E << (static_cast<int>(channel) * 4));
}

template <typename T>
inline bool GeneralPurposeTimer<T>::isCaptureComapreComplementaryOutputEnabled(
    TimerUtils::Channel channel)
{
    return timer->CCER & (TIM_CCER_CC1NE << (static_cast<int>(channel) * 4));
}

template <typename T>
inline void GeneralPurposeTimer<T>::setCaptureComparePolarity(
    TimerUtils::Channel channel, TimerUtils::OutputComparePolarity polarity)
{
    timer->CCER |= static_cast<uint16_t>(polarity)
                   << (1 + static_cast<int>(channel) * 4);
}

template <typename T>
inline void GeneralPurposeTimer<T>::setCaptureCompareComplementaryPolarity(
    TimerUtils::Channel channel, TimerUtils::OutputComparePolarity polarity)
{
    timer->CCER |= static_cast<uint16_t>(polarity)
                   << (3 + static_cast<int>(channel) * 4);
}

template <typename T>
inline void GeneralPurposeTimer<T>::setCaptureCompareRegister(
    TimerUtils::Channel channel, T value)
{
    switch (channel)
    {
        case TimerUtils::Channel::CHANNEL_1:
            timer->CCR1 = value;
            break;
        case TimerUtils::Channel::CHANNEL_2:
            timer->CCR2 = value;
            break;
        case TimerUtils::Channel::CHANNEL_3:
            timer->CCR3 = value;
            break;
        case TimerUtils::Channel::CHANNEL_4:
            timer->CCR4 = value;
            break;
    }
}

template <typename T>
inline T GeneralPurposeTimer<T>::readCaptureCompareRegister(
    TimerUtils::Channel channel)
{
    switch (channel)
    {
        case TimerUtils::Channel::CHANNEL_1:
            return timer->CCR1;
        case TimerUtils::Channel::CHANNEL_2:
            return timer->CCR2;
        case TimerUtils::Channel::CHANNEL_3:
            return timer->CCR3;
        case TimerUtils::Channel::CHANNEL_4:
            return timer->CCR4;
    }

    return 0;
}

template <typename T>
inline void GeneralPurposeTimer<T>::clearTriggerInterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_TIF;
}

template <typename T>
inline void GeneralPurposeTimer<T>::clearCaptureCompareInterruptFlag(
    TimerUtils::Channel channel, TIM_TypeDef *timer)
{
    timer->SR &= ~(TIM_SR_CC1IF << static_cast<int>(channel));
}

}  // namespace Boardcore
