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

using namespace Boardcore::TimerUtils;

namespace Boardcore
{

PWM::PWM(TIM_TypeDef* const pulseTimer, uint16_t pulseFrequency)
    : pulseTimer(pulseTimer), pulseFrequency(pulseFrequency)
{
    // Erase the previous timer configuration
    this->pulseTimer.reset();

    configureTimer();

    // Keep the timer always enabled
    this->pulseTimer.enable();
}

PWM::~PWM() { pulseTimer.reset(); }

void PWM::setFrequency(uint16_t pulseFrequency)
{
    this->pulseFrequency = pulseFrequency;
    configureTimer();
}

void PWM::setDutyCycleResolution(uint16_t dutyCycleResolution)
{
    this->dutyCycleResolution = dutyCycleResolution;
    configureTimer();
}

void PWM::enableChannel(Channel channel, Polarity polarity)
{
    pulseTimer.setOutputCompareMode(channel, OutputCompareMode::PWM_MODE_1);
    pulseTimer.setCaptureComparePolarity(
        channel, static_cast<OutputComparePolarity>(polarity));

    // This will ensure that the duty cycle will change only at the next period
    pulseTimer.enableCaptureComparePreload(channel);

    pulseTimer.enableCaptureCompareOutput(channel);
}

void PWM::disableChannel(Channel channel)
{
    pulseTimer.disableCaptureCompareOutput(channel);
}

bool PWM::isChannelEnabled(Channel channel)
{
    return pulseTimer.isCaptureCompareOutputEnabled(channel);
}

void PWM::setDutyCycle(Channel channel, float dutyCycle)
{
    if (dutyCycle >= 0 && dutyCycle <= 1)
    {
        pulseTimer.setCaptureCompareRegister(
            channel,
            static_cast<uint16_t>(
                dutyCycle * pulseTimer.readAutoReloadRegister() + 0.5));
    }
}

float PWM::getDutyCycle(TimerUtils::Channel channel)
{
    return static_cast<float>(pulseTimer.readCaptureCompareRegister(channel)) /
           static_cast<float>(pulseTimer.readAutoReloadRegister());
}

GP16bitTimer& PWM::getTimer() { return pulseTimer; }

void PWM::configureTimer()
{
    pulseTimer.setFrequency(dutyCycleResolution * pulseFrequency);
    pulseTimer.setAutoReloadRegister(getFrequency(pulseTimer.getTimer()) /
                                     pulseFrequency);

    // Force the timer to update its configuration
    pulseTimer.generateUpdate();
}

}  // namespace Boardcore
