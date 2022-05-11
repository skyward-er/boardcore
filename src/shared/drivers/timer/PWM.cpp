/* Copyright (c) 2017-2021 Skyward Experimental Rocketry
 * Authors: Andrea Palumbo, Luca Erbetta, Alberto Nidasio
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

#include "PWM.h"

namespace Boardcore
{

PWM::PWM(TIM_TypeDef* const timer, unsigned int pwmFrequency,
         unsigned int dutyCycleResolution)
    : timer(timer), pwmFrequency(pwmFrequency),
      dutyCycleResolution(dutyCycleResolution)
{
    // Erase the previous timer configuration
    this->timer.reset();

    // Set up and enable the timer
    setTimerConfiguration();
}

PWM::~PWM() { timer.reset(); }

void PWM::setFrequency(unsigned int pwmFrequency)
{
    this->pwmFrequency = pwmFrequency;
    setTimerConfiguration();
}

void PWM::setDutyCycleResolution(unsigned int dutyCycleResolution)
{
    this->dutyCycleResolution = dutyCycleResolution;
    setTimerConfiguration();
}

void PWM::enableChannel(TimerUtils::Channel channel, Polarity polarity)
{
    timer.setOutputCompareMode(channel,
                               TimerUtils::OutputCompareMode::PWM_MODE_1);
    timer.setCaptureComparePolarity(
        channel, static_cast<TimerUtils::OutputComparePolarity>(polarity));

    // This will ensure that the duty cycle will change only at the next period
    timer.enableCaptureComparePreload(channel);

    timer.enableCaptureCompareOutput(channel);
}

void PWM::disableChannel(TimerUtils::Channel channel)
{
    timer.disableCaptureCompareOutput(channel);
}

bool PWM::isChannelEnabled(TimerUtils::Channel channel)
{
    return timer.isCaptureComapreOutputEnabled(channel);
}

void PWM::setDutyCycle(TimerUtils::Channel channel, float dutyCycle)
{
    if (dutyCycle >= 0 && dutyCycle <= 1)
        timer.setCaptureCompareRegister(
            channel, static_cast<uint16_t>(
                         dutyCycle * timer.readAutoReloadRegister() + 0.5));
}

float PWM::getDutyCycle(TimerUtils::Channel channel)
{
    return static_cast<float>(timer.readCaptureCompareRegister(channel)) /
           static_cast<float>(timer.readAutoReloadRegister());
}

GP16bitTimer& PWM::getTimer() { return timer; }

void PWM::setTimerConfiguration()
{
    timer.disable();
    timer.setCounter(0);

    timer.setFrequency(dutyCycleResolution * pwmFrequency);

    timer.setAutoReloadRegister(TimerUtils::getFrequency(timer.getTimer()) /
                                pwmFrequency);

    // Force the timer to update its configuration
    timer.generateUpdate();

    timer.enable();
}

}  // namespace Boardcore
