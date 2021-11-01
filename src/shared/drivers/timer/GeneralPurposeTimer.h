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
#include <miosix.h>

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
class GeneralPurposeTimer : public BasicTimer
{
public:
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
     * @brief Enable capture/compare 1 interrupt.
     */
    void enableCaptureCompare1Interrupt();

    /**
     * @brief Disable capture/compare 1 interrupt.
     */
    void disableCaptureCompare1Interrupt();

    /**
     * @brief Enable capture/compare 1 interrupt.
     */
    void enableCaptureCompare1DMARequest();

    /**
     * @brief Disable capture/compare 1 interrupt.
     */
    void disableCaptureCompare1DMARequest();

    /**
     * @brief Enable capture/compare 2 interrupt.
     */
    void enableCaptureCompare2Interrupt();

    /**
     * @brief Disable capture/compare 2 interrupt.
     */
    void disableCaptureCompare2Interrupt();

    /**
     * @brief Enable capture/compare 2 interrupt.
     */
    void enableCaptureCompare2DMARequest();

    /**
     * @brief Disable capture/compare 2 interrupt.
     */
    void disableCaptureCompare2DMARequest();

    /**
     * @brief Enable capture/compare 3 interrupt.
     */
    void enableCaptureCompare3Interrupt();

    /**
     * @brief Disable capture/compare 3 interrupt.
     */
    void disableCaptureCompare3Interrupt();

    /**
     * @brief Enable capture/compare 3 interrupt.
     */
    void enableCaptureCompare3DMARequest();

    /**
     * @brief Disable capture/compare 3 interrupt.
     */
    void disableCaptureCompare3DMARequest();

    /**
     * @brief Enable capture/compare 4 interrupt.
     */
    void enableCaptureCompare4Interrupt();

    /**
     * @brief Disable capture/compare 4 interrupt.
     */
    void disableCaptureCompare4Interrupt();

    /**
     * @brief Enable capture/compare 4 interrupt.
     */
    void enableCaptureCompare4DMARequest();

    /**
     * @brief Disable capture/compare 4 interrupt.
     */
    void disableCaptureCompare4DMARequest();

    /**
     * @brief The TIF flag is set in the TIMx_SR register. Related interrupt can
     * occur if enabled.
     */
    void generateTrigger();

    /**
     * @brief A capture/compare event is generated on channel 1.
     */
    void generateCaptureCompare1Event();

    /**
     * @brief A capture/compare event is generated on channel 2.
     */
    void generateCaptureCompare2Event();

    /**
     * @brief The capture/compare 1 register is buffered.
     *
     * This means that the shadow register is used and the capture/compare 1
     * value will became active at the next UEV.
     */
    void enableCaptureCompare1Preload();

    /**
     * @brief Tha capture/compare register 1 is not buffered.
     *
     * This means that when you change the capture/compare 1 register, its value
     * is taken into account immediately.
     */
    void disableCaptureCompare1Preload();

    /**
     * @brief The capture/compare 2 register is buffered.
     *
     * This means that the shadow register is used and the capture/compare 2
     * value will became active at the next UEV.
     */
    void enableCaptureCompare2Preload();

    /**
     * @brief The capture/compare register 2 is not buffered.
     *
     * This means that when you change the capture/compare 2 register, its value
     * is taken into account immediately.
     */
    void disableCaptureCompare2Preload();

    /**
     * @brief The capture/compare 3 register is buffered.
     *
     * This means that the shadow register is used and the capture/compare 3
     * value will became active at the next UEV.
     */
    void enableCaptureCompare3Preload();

    /**
     * @brief The capture/compare register 3 is not buffered.
     *
     * This means that when you change the capture/compare 3 register, its value
     * is taken into account immediately.
     */
    void disableCaptureCompare3Preload();

    /**
     * @brief The capture/compare 4 register is buffered.
     *
     * This means that the shadow register is used and the capture/compare 4
     * value will became active at the next UEV.
     */
    void enableCaptureCompare4Preload();

    /**
     * @brief The capture/compare register 4 is not buffered.
     *
     * This means that when you change the capture/compare 4 register, its value
     * is taken into account immediately.
     */
    void disableCaptureCompare4Preload();

    /**
     * @brief Sets the output/compare 1 mode.
     */
    void setOutputCompare1Mode(OutputCompareMode mode);

    /**
     * @brief Sets the output/compare 2 mode.
     */
    void setOutputCompare2Mode(OutputCompareMode mode);

    /**
     * @brief Sets the output/compare 3 mode.
     */
    void setOutputCompare3Mode(OutputCompareMode mode);

    /**
     * @brief Sets the output/compare 4 mode.
     */
    void setOutputCompare4Mode(OutputCompareMode mode);

    /**
     * @brief Enables capture/compare 1 channel.
     */
    void enableCaptureCompare1Output();

    /**
     * @brief Disables capture/compare 1 channel.
     */
    void disableCaptureCompare1Output();

    /**
     * @brief Enables capture/compare 2 channel.
     */
    void enableCaptureCompare2Output();

    /**
     * @brief Disables capture/compare 2 channel.
     */
    void disableCaptureCompare2Output();

    /**
     * @brief Enables capture/compare 3 channel.
     */
    void enableCaptureCompare3Output();

    /**
     * @brief Disables capture/compare 3 channel.
     */
    void disableCaptureCompare3Output();

    /**
     * @brief Enables capture/compare 4 channel.
     */
    void enableCaptureCompare4Output();

    /**
     * @brief Disables capture/compare 4 channel.
     */
    void disableCaptureCompare4Output();

    /**
     * @brief Changes capture/compare 1 polarity.
     */
    void setCaptureCompare1Polarity(OutputComparePolarity polarity);

    /**
     * @brief Changes capture/compare 2 polarity.
     */
    void setCaptureCompare2Polarity(OutputComparePolarity polarity);

    /**
     * @brief Changes capture/compare 3 polarity.
     */
    void setCaptureCompare3Polarity(OutputComparePolarity polarity);

    /**
     * @brief Changes capture/compare 4 polarity.
     */
    void setCaptureCompare4Polarity(OutputComparePolarity polarity);

    /**
     * @brief Sets the timer capture/compare 1 value.
     */
    void setCaptureCompare1Register(uint16_t value);

    /**
     * @brief Sets the timer capture/compare 2 value.
     */
    void setCaptureCompare2Register(uint16_t value);

    /**
     * @brief Sets the timer capture/compare 3 value.
     */
    void setCaptureCompare3Register(uint16_t value);

    /**
     * @brief Sets the timer capture/compare 4 value.
     */
    void setCaptureCompare4Register(uint16_t value);

    /**
     * @brief Clears the trigger interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearTriggerInterruptFlag(TIM_TypeDef *timer);

    /**
     * @brief Clears capture/compare 1 interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearCaptureCompare1InterruptFlag(TIM_TypeDef *timer);

    /**
     * @brief Clears capture/compare 2 interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearCaptureCompare2InterruptFlag(TIM_TypeDef *timer);

    /**
     * @brief Clears capture/compare 3 interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearCaptureCompare3InterruptFlag(TIM_TypeDef *timer);

    /**
     * @brief Clears capture/compare 4 interrupt flag.
     *
     * @param timer Timer to use.
     */
    static void clearCaptureCompare4InterruptFlag(TIM_TypeDef *timer);
};

inline GeneralPurposeTimer::GeneralPurposeTimer(TIM_TypeDef *timer)
    : BasicTimer(timer)
{
}

inline void GeneralPurposeTimer::reset()
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

inline void GeneralPurposeTimer::setMasterMode(MasterMode masterMode)
{
    // First clear the configuration
    timer->CR2 &= ~TIM_CR2_MMS;

    // Set the new value
    timer->CR2 |= (uint16_t)masterMode;
}

inline void GeneralPurposeTimer::setSlaveMode(SlaveMode slaveMode)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_SMS;

    // Set the new value
    timer->SMCR |= (uint16_t)slaveMode;
}

inline void GeneralPurposeTimer::setTriggerSource(TriggerSource triggerSource)
{
    // First clear the configuration
    timer->SMCR &= ~TIM_SMCR_TS;

    // Set the new value
    timer->SMCR |= (uint16_t)triggerSource;
}

inline void GeneralPurposeTimer::enableTriggerInterrupt()
{
    timer->DIER |= TIM_DIER_TIE;
}

inline void GeneralPurposeTimer::disableTriggerInterrupt()
{
    timer->DIER &= ~TIM_DIER_TIE;
}

inline void GeneralPurposeTimer::enableCaptureCompare1Interrupt()
{
    timer->DIER |= TIM_DIER_CC1IE;
}

inline void GeneralPurposeTimer::disableCaptureCompare1Interrupt()
{
    timer->DIER &= ~TIM_DIER_CC1IE;
}

inline void GeneralPurposeTimer::enableCaptureCompare1DMARequest()
{
    timer->DIER |= TIM_DIER_CC1DE;
}

inline void GeneralPurposeTimer::disableCaptureCompare1DMARequest()
{
    timer->DIER &= ~TIM_DIER_CC1DE;
}

inline void GeneralPurposeTimer::enableCaptureCompare2Interrupt()
{
    timer->DIER |= TIM_DIER_CC2IE;
}

inline void GeneralPurposeTimer::disableCaptureCompare2Interrupt()
{
    timer->DIER &= ~TIM_DIER_CC2IE;
}

inline void GeneralPurposeTimer::enableCaptureCompare2DMARequest()
{
    timer->DIER |= TIM_DIER_CC2DE;
}

inline void GeneralPurposeTimer::disableCaptureCompare2DMARequest()
{
    timer->DIER &= ~TIM_DIER_CC2DE;
}

inline void GeneralPurposeTimer::enableCaptureCompare3Interrupt()
{
    timer->DIER |= TIM_DIER_CC3IE;
}

inline void GeneralPurposeTimer::disableCaptureCompare3Interrupt()
{
    timer->DIER &= ~TIM_DIER_CC3IE;
}

inline void GeneralPurposeTimer::enableCaptureCompare3DMARequest()
{
    timer->DIER |= TIM_DIER_CC3DE;
}

inline void GeneralPurposeTimer::disableCaptureCompare3DMARequest()
{
    timer->DIER &= ~TIM_DIER_CC3DE;
}

inline void GeneralPurposeTimer::enableCaptureCompare4Interrupt()
{
    timer->DIER |= TIM_DIER_CC4IE;
}

inline void GeneralPurposeTimer::disableCaptureCompare4Interrupt()
{
    timer->DIER &= ~TIM_DIER_CC4IE;
}

inline void GeneralPurposeTimer::enableCaptureCompare4DMARequest()
{
    timer->DIER |= TIM_DIER_CC4DE;
}

inline void GeneralPurposeTimer::disableCaptureCompare4DMARequest()
{
    timer->DIER &= ~TIM_DIER_CC4DE;
}

inline void GeneralPurposeTimer::enableCaptureCompare1Preload()
{
    timer->CCMR1 |= TIM_CCMR1_OC1PE;
}

inline void GeneralPurposeTimer::disableCaptureCompare1Preload()
{
    timer->CCMR1 &= ~TIM_CCMR1_OC1PE;
}

inline void GeneralPurposeTimer::enableCaptureCompare2Preload()
{
    timer->CCMR1 |= TIM_CCMR1_OC2PE;
}

inline void GeneralPurposeTimer::disableCaptureCompare2Preload()
{
    timer->CCMR1 &= ~TIM_CCMR1_OC2PE;
}

inline void GeneralPurposeTimer::enableCaptureCompare3Preload()
{
    timer->CCMR2 |= TIM_CCMR2_OC3PE;
}

inline void GeneralPurposeTimer::disableCaptureCompare3Preload()
{
    timer->CCMR2 &= ~TIM_CCMR2_OC3PE;
}

inline void GeneralPurposeTimer::enableCaptureCompare4Preload()
{
    timer->CCMR2 |= TIM_CCMR2_OC4PE;
}

inline void GeneralPurposeTimer::disableCaptureCompare4Preload()
{
    timer->CCMR2 &= ~TIM_CCMR2_OC4PE;
}

inline void GeneralPurposeTimer::setOutputCompare1Mode(OutputCompareMode mode)
{
    // First clear the configuration
    timer->CCMR1 &= ~TIM_CCMR1_OC1M;

    // Set the new value
    timer->CCMR1 |= (uint16_t)mode << 4;
}

inline void GeneralPurposeTimer::setOutputCompare2Mode(OutputCompareMode mode)
{
    // First clear the configuration
    timer->CCMR1 &= ~TIM_CCMR1_OC2M;

    // Set the new value
    timer->CCMR1 |= (uint16_t)mode << 12;
}

inline void GeneralPurposeTimer::setOutputCompare3Mode(OutputCompareMode mode)
{
    // First clear the configuration
    timer->CCMR2 &= ~TIM_CCMR2_OC3M;

    // Set the new value
    timer->CCMR2 |= (uint16_t)mode << 4;
}

inline void GeneralPurposeTimer::setOutputCompare4Mode(OutputCompareMode mode)
{
    // First clear the configuration
    timer->CCMR2 &= ~TIM_CCMR2_OC4M;

    // Set the new value
    timer->CCMR2 |= (uint16_t)mode << 12;
}

inline void GeneralPurposeTimer::enableCaptureCompare1Output()
{
    timer->CCER |= TIM_CCER_CC1E;
}

inline void GeneralPurposeTimer::disableCaptureCompare1Output()
{
    timer->CCER &= ~TIM_CCER_CC1E;
}

inline void GeneralPurposeTimer::enableCaptureCompare2Output()
{
    timer->CCER |= TIM_CCER_CC2E;
}

inline void GeneralPurposeTimer::disableCaptureCompare2Output()
{
    timer->CCER &= ~TIM_CCER_CC2E;
}

inline void GeneralPurposeTimer::enableCaptureCompare3Output()
{
    timer->CCER |= TIM_CCER_CC3E;
}

inline void GeneralPurposeTimer::disableCaptureCompare3Output()
{
    timer->CCER &= ~TIM_CCER_CC3E;
}

inline void GeneralPurposeTimer::enableCaptureCompare4Output()
{
    timer->CCER |= TIM_CCER_CC4E;
}

inline void GeneralPurposeTimer::disableCaptureCompare4Output()
{
    timer->CCER &= ~TIM_CCER_CC4E;
}

inline void GeneralPurposeTimer::setCaptureCompare1Polarity(
    OutputComparePolarity polarity)
{
    timer->CCER |= (uint16_t)polarity << 1;
}

inline void GeneralPurposeTimer::setCaptureCompare2Polarity(
    OutputComparePolarity polarity)
{
    timer->CCER |= (uint16_t)polarity << 5;
}

inline void GeneralPurposeTimer::setCaptureCompare3Polarity(
    OutputComparePolarity polarity)
{
    timer->CCER |= (uint16_t)polarity << 9;
}

inline void GeneralPurposeTimer::setCaptureCompare4Polarity(
    OutputComparePolarity polarity)
{
    timer->CCER |= (uint16_t)polarity << 13;
}

inline void GeneralPurposeTimer::setCaptureCompare1Register(uint16_t value)
{
    timer->CCR1 = value;
}

inline void GeneralPurposeTimer::setCaptureCompare2Register(uint16_t value)
{
    timer->CCR2 = value;
}

inline void GeneralPurposeTimer::setCaptureCompare3Register(uint16_t value)
{
    timer->CCR3 = value;
}

inline void GeneralPurposeTimer::setCaptureCompare4Register(uint16_t value)
{
    timer->CCR4 = value;
}

inline void GeneralPurposeTimer::clearTriggerInterruptFlag(TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_TIF;
}

inline void GeneralPurposeTimer::clearCaptureCompare1InterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_CC1IF;
}

inline void GeneralPurposeTimer::clearCaptureCompare2InterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_CC2IF;
}

inline void GeneralPurposeTimer::clearCaptureCompare3InterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_CC3IF;
}

inline void GeneralPurposeTimer::clearCaptureCompare4InterruptFlag(
    TIM_TypeDef *timer)
{
    timer->SR &= ~TIM_SR_CC4IF;
}