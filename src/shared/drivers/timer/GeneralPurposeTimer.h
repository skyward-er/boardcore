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

#include <drivers/timer/BasicTimer.h>

#include <type_traits>

namespace timer
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
 *   - Trigger event (counter start, stop, initializationm or count by internal
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
 * For a more exhautive explanation and feature list study the reference manual!
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
class GeneralPurposeTimer : public BasicTimer
{
public:
    static_assert(std::is_same<T, uint16_t>::value ||
                      std::is_same<T, uint32_t>::value,
                  "Type must be either uint16_t or uint32_t.");

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
        ITR3 = TIM_SMCR_TS_0 | TIM_SMCR_TS_1,

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

    /**
     * @brief Create a GeneralPurposeTimer object. Note that this does not
     * resets the timer configuration.
     */
    GeneralPurposeTimer(TIM_TypeDef *timer);

    /**
     * @brief Resets the timer configuration to the default state.
     *
     * TODO: write the default state
     */
    void reset();

    /**
     * @brief Reads the counter value.
     *
     * @return Counter value
     */
    T readCounter();

    /**
     * @brief Sets the timer counter value.
     *
     * @param counterValue Counter value to set.
     */
    void setCounter(T counterValue);

    /**
     * @brief Reads the timer auto-reload register.
     *
     * @return Tiemr auto-reload register value.
     */
    T readAutoReloadRegister();

    /**
     * @brief Changes the auto-reload register.
     *
     * @param autoReloadValue New auto-reload register value.
     */
    void setAutoReloadRegister(T autoReloadValue);

    /**
     * @brief Changes the timer master mode.
     *
     * @param masterMode New timer master mode.
     */
    void setMasterMode(MasterMode masterMode);

    /**
     * @brief Changes the timer slave mode.
     *
     * @param slaveMode New timer slave mode.
     */
    void setSlaveMode(SlaveMode slaveMode);

    /**
     * @brief Changes the trigger source.
     */
    void setTriggerSource(TriggerSource triggerSource);

    /**
     * @brief Enable trigger interrupt.
     */
    void enableTriggerInterrupt();

    /**
     * @brief Disable trigger interrupt.
     */
    void disableTriggerInterrupt();

    /**
     * @brief Enable capture/compare channel interrupt.
     */
    template <int C>
    void enableCaptureCompareInterrupt();

    /**
     * @brief Disable capture/compare channel interrupt.
     */
    template <int C>
    void disableCaptureCompareInterrupt();

    /**
     * @brief Enable capture/compare channel interrupt.
     */
    template <int C>
    void enableCaptureCompareDMARequest();

    /**
     * @brief Disable capture/compare channel interrupt.
     */
    template <int C>
    void disableCaptureCompareDMARequest();

    /**
     * @brief The TIF flag is set in the TIMx_SR register. Related interrupt can
     * occur if enabled.
     */
    void generateTrigger();

    /**
     * @brief A capture/compare event is generated on channel.
     */
    template <int C>
    void generateCaptureCompareEvent();

    /**
     * @brief The capture/compare register is buffered.
     *
     * This means that the shadow register is used and the capture/compare
     * value will became active at the next UEV.
     */
    template <int C>
    void enableCaptureComparePreload();

    /**
     * @brief Tha capture/compare register is not buffered.
     *
     * This means that when you change the capture/compare  register, its value
     * is taken into account immediately.
     */
    template <int C>
    void disableCaptureComparePreload();

    /**
     * @brief Sets the output/compare mode.
     */
    template <int C>
    void setOutputCompareMode(OutputCompareMode mode);

    /**
     * @brief Enables capture/compare channel.
     */
    template <int C>
    void enableCaptureCompareOutput();

    /**
     * @brief Disables capture/compare channel.
     */
    template <int C>
    void disableCaptureCompareOutput();

    /**
     * @brief Changes capture/compare polarity.
     */
    template <int C>
    void setCaptureComparePolarity(OutputComparePolarity polarity);

    /**
     * @brief Sets the timer capture/compare value.
     */
    template <int C>
    void setCaptureCompareRegister(T value);

    /**
     * @brief Clears the trigger interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearTriggerInterruptFlag(TIM_TypeDef *timer);

    /**
     * @brief Clears capture/compare interrupt flag.
     *
     * @param timer Timer to use.
     */
    template <int C>
    static void clearCaptureCompareInterruptFlag(TIM_TypeDef *timer);
};

}  // namespace timer

template <typename T>
inline timer::GeneralPurposeTimer<T>::GeneralPurposeTimer(TIM_TypeDef *timer)
    : BasicTimer(timer)
{
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::reset()
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
    timer->ARR   = (uint32_t)0xFFFFFFFFF;
    timer->CCR1  = 0;
    timer->CCR2  = 0;
    timer->CCR3  = 0;
    timer->CCR4  = 0;
    timer->DCR   = 0;
    timer->DMAR  = 0;
    timer->OR    = 0;
}

template <typename T>
inline T timer::GeneralPurposeTimer<T>::readCounter()
{
    return timer->CNT;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::setCounter(T counterValue)
{
    timer->CNT = counterValue;
}

template <typename T>
inline T timer::GeneralPurposeTimer<T>::readAutoReloadRegister()
{
    return timer->ARR;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::setAutoReloadRegister(
    T autoReloadValue)
{
    timer->ARR = autoReloadValue;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::setMasterMode(MasterMode masterMode)
{
    // First clear the configuration
    timer->CR2 &= ~TIM_CR2_MMS;

    // Set the new value
    timer->CR2 |= (uint16_t)masterMode;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::setSlaveMode(SlaveMode slaveMode)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_SMS;

    // Set the new value
    timer->SMCR |= (uint16_t)slaveMode;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::setTriggerSource(
    TriggerSource triggerSource)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_TS;

    // Set the new value
    timer->SMCR |= (uint16_t)triggerSource;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::enableTriggerInterrupt()
{
    timer->DIER |= TIM_DIER_TIE;
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::disableTriggerInterrupt()
{
    timer->DIER &= ~TIM_DIER_TIE;
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::enableCaptureCompareInterrupt()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->DIER |= TIM_DIER_CC1IE << (C - 1);
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::disableCaptureCompareInterrupt()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->DIER &= ~(TIM_DIER_CC1IE << (C - 1));
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::enableCaptureCompareDMARequest()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->DIER |= TIM_DIER_CC1DE << (C - 1);
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::disableCaptureCompareDMARequest()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->DIER &= ~(TIM_DIER_CC1DE << (C - 1));
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::generateTrigger()
{
    timer->EGR |= TIM_EGR_TG;
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::generateCaptureCompareEvent()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->EGR |= TIM_EGR_CC1G << (C - 1);
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::enableCaptureComparePreload()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    switch (C)
    {
        case 1:
            timer->CCMR1 |= TIM_CCMR1_OC1PE;
            break;
        case 2:
            timer->CCMR1 |= TIM_CCMR1_OC2PE;
            break;
        case 3:
            timer->CCMR2 |= TIM_CCMR2_OC3PE;
            break;
        case 4:
            timer->CCMR2 |= TIM_CCMR2_OC4PE;
            break;
    }
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::disableCaptureComparePreload()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    switch (C)
    {
        case 1:
            timer->CCMR1 &= ~TIM_CCMR1_OC1PE;
            break;
        case 2:
            timer->CCMR1 &= ~TIM_CCMR1_OC2PE;
            break;
        case 3:
            timer->CCMR2 &= ~TIM_CCMR2_OC3PE;
            break;
        case 4:
            timer->CCMR2 &= ~TIM_CCMR2_OC4PE;
            break;
    }
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::setOutputCompareMode(
    OutputCompareMode mode)
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    switch (C)
    {
        case 1:
            // First clear the configuration
            timer->CCMR1 &= ~TIM_CCMR1_OC1M;

            // Set the new value
            timer->CCMR1 |= (uint16_t)mode << 4;
            break;
        case 2:
            // First clear the configuration
            timer->CCMR1 &= ~TIM_CCMR1_OC2M;

            // Set the new value
            timer->CCMR1 |= (uint16_t)mode << 12;
            break;
        case 3:
            // First clear the configuration
            timer->CCMR2 &= ~TIM_CCMR2_OC3M;

            // Set the new value
            timer->CCMR2 |= (uint16_t)mode << 4;
            break;
        case 4:
            // First clear the configuration
            timer->CCMR2 &= ~TIM_CCMR2_OC4M;

            // Set the new value
            timer->CCMR2 |= (uint16_t)mode << 12;
            break;
    }
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::enableCaptureCompareOutput()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->CCER |= TIM_CCER_CC1E << ((C - 1) * 4);
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::disableCaptureCompareOutput()
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->CCER &= ~(TIM_CCER_CC1E << ((C - 1) * 4));
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::setCaptureComparePolarity(
    OutputComparePolarity polarity)
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->CCER |= (uint16_t)polarity << (1 + (C - 1) * 4);
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::setCaptureCompareRegister(T value)
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    switch (C)
    {
        case 1:
            timer->CCR1 = value;
            break;
        case 2:
            timer->CCR2 = value;
            break;
        case 3:
            timer->CCR3 = value;
            break;
        case 4:
            timer->CCR4 = value;
            break;
    }
}

template <typename T>
inline void timer::GeneralPurposeTimer<T>::clearTriggerInterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_TIF;
}

template <typename T>
template <int C>
inline void timer::GeneralPurposeTimer<T>::clearCaptureCompareInterruptFlag(
    TIM_TypeDef *timer)
{
    static_assert(C >= 1 && C <= 4, "Channel must be betwen 1 and 4");

    timer->SR &= ~(TIM_SR_CC1IF << (C - 1));
}
