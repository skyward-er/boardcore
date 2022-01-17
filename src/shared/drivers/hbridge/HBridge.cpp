/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include "HBridge.h"

namespace Boardcore
{

HBridge::HBridge(GpioPin inhibitPin, TIM_TypeDef* timer,
                 TimerUtils::Channel channel, unsigned int frequency,
                 float dutyCycle, unsigned int disableDelayMs)
    : inhibitPin(inhibitPin), pwm(timer, frequency), channel(channel),
      frequency(frequency), dutyCycle(dutyCycle), disableDelayMs(disableDelayMs)
{
    inhibitPin.low();
}

HBridge::~HBridge() { disable(); }

void HBridge::enable()
{
    pwm.setDutyCycle(channel, dutyCycle);
    pwm.enableChannel(channel);

    inhibitPin.high();
}

void HBridge::disable()
{
    pwm.disableChannel(channel);

    Thread::sleep(disableDelayMs);

    inhibitPin.low();
}

bool HBridge::isEnabled() { return pwm.isChannelEnabled(channel); }

void HBridge::setDutyCycle(float dutyCycle)
{
    this->dutyCycle = dutyCycle;

    pwm.setDutyCycle(channel, dutyCycle);
}

void HBridge::testDutyCycle(float testDutyCycle)
{
    pwm.setDutyCycle(channel, testDutyCycle);
}

}  // namespace Boardcore
