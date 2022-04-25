/* Copyright (c) 2022 Skyward Experimental Rocketry
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

namespace Boardcore
{

/**
 * @brief This driver does not provide a square wave signal but insted is a
 * simple utility that provides long PWM signals to make the buzzer beep
 * on and off.
 */
class Buzzer
{
public:
    /**
     * Note: The timer must be a General Purpose Timer.
     *
     * @param timer Timer used to provide the alternating on and off signal.
     * @param channel Timer channel to output the signal.
     */
    Buzzer(TIM_TypeDef *timer, TimerUtils::Channel channel);

    /**
     * @brief Turns on the buzzer.
     *
     * Note: Keeps the timer off.
     */
    void on();

    /**
     * @brief Turns off the buzzer.
     *
     * Note: disables and resets the timer.
     */
    void off();

    /**
     * @brief Turns on the buzzer for the specified amount of time.
     *
     * Note: This function wait for the given duration with Thread::sleep.
     *
     * @param ms Beep duration in milliseconds.
     */
    void oneTimeToggle(uint16_t ms);

    /**
     * @brief Turns on and off the buzzer indefinitely with the given timings.
     *
     * @param onTime Buzzer on time [ms].
     * @param offTime Buzzer off time [ms].
     */
    void continuoslyToggle(uint16_t onTime, uint16_t offTime);

private:
    GP16bitTimer timer;
    TimerUtils::Channel channel;
};

inline Buzzer::Buzzer(TIM_TypeDef *timer, TimerUtils::Channel channel)
    : timer(timer), channel(channel)
{
    // this->timer.setPrescaler(
    //     TimerUtils::computePrescalerValue(timer, frequency * 2));
    // this->timer.setAutoReloadRegister(1);
    // this->timer.setOutputCompareMode(channel,
    //                                  TimerUtils::OutputCompareMode::TOGGLE);
    // this->timer.setCaptureCompareRegister(channel, 1);
    // this->timer.generateUpdate();
    // this->timer.enableCaptureCompareOutput(channel);
    // this->timer.enableCaptureCompareComplementaryOutput(channel);
}

inline void Buzzer::on()
{
    // First turn off and reset the timer
    off();

    // Set the polarity of the channel to low to make the output high
    timer.setOutputCompareMode(channel,
                               TimerUtils::OutputCompareMode::FORCE_ACTIVE);
    timer.setCaptureComparePolarity(
        channel, TimerUtils::OutputComparePolarity::ACTIVE_LOW);
    timer.enableCaptureCompareOutput(channel);
}

inline void Buzzer::off()
{
    timer.disable();
    timer.reset();
}

inline void Buzzer::oneTimeToggle(uint16_t ms)
{
    if (ms == 0)
        return;

    // First turn off and reset the timer
    off();

    // Set the prescaler with a 10KHz frequency
    timer.setPrescaler(
        TimerUtils::computePrescalerValue(timer.getTimer(), 10000));

    // Make the timer reload after the specified amount of millisseconds
    timer.setAutoReloadRegister(ms * 10);
    timer.setCaptureCompareRegister(channel, 1);

    timer.setOutputCompareMode(channel,
                               TimerUtils::OutputCompareMode::PWM_MODE_2);
    timer.setCaptureComparePolarity(
        channel, TimerUtils::OutputComparePolarity::ACTIVE_LOW);
    timer.enableCaptureCompareOutput(channel);

    // Update the configuration
    timer.generateUpdate();

    // Use One Pulse Mode to stop the timer after the first update event
    timer.enableOnePulseMode();
    timer.enable();
}

inline void Buzzer::continuoslyToggle(uint16_t onTime, uint16_t offTime)
{
    if (onTime == 0 || offTime == 0)
        return;

    // First turn off and reset the timer
    off();

    // Set the prescaler with a 10KHz frequency
    timer.setPrescaler(
        TimerUtils::computePrescalerValue(timer.getTimer(), 10000));

    // Make the timer reload after the specified amount of millisseconds
    timer.setAutoReloadRegister((onTime + offTime) * 10);
    timer.setCaptureCompareRegister(channel, onTime * 10);

    timer.setOutputCompareMode(channel,
                               TimerUtils::OutputCompareMode::PWM_MODE_1);
    timer.setCaptureComparePolarity(
        channel, TimerUtils::OutputComparePolarity::ACTIVE_LOW);
    timer.enableCaptureCompareOutput(channel);

    // Update the configuration
    timer.generateUpdate();

    timer.enable();
}

}  // namespace Boardcore
