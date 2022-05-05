/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include "Servo.h"

#include <drivers/timer/TimestampTimer.h>

#include "miosix.h"

namespace Boardcore
{

#ifndef COMPILE_FOR_HOST

Servo::Servo(TIM_TypeDef* const timer, TimerUtils::Channel pwmChannel,
             unsigned int minPulse, unsigned int maxPulse,
             unsigned int frequency)
    : pwm(timer, frequency), pwmChannel(pwmChannel), minPulse(minPulse),
      maxPulse(maxPulse), frequency(frequency)
{
    setPosition(0);
}

void Servo::enable() { pwm.enableChannel(pwmChannel); }

void Servo::disable() { pwm.disableChannel(pwmChannel); }

#else

Servo::Servo(unsigned int minPulse, unsigned int maxPulse,
             unsigned int frequency)
    : minPulse(minPulse), maxPulse(maxPulse), frequency(frequency)
{
    setPosition(0);
}

void Servo::enable() {}

void Servo::disable() {}

#endif

void Servo::setPosition(float position)
{
    float pulse = minPulse + (maxPulse - minPulse) * position;

    float dutyCycle = pulse * frequency / 1000000.0f;

#ifndef COMPILE_FOR_HOST
    pwm.setDutyCycle(pwmChannel, dutyCycle);
#else
    this->dutyCycle = dutyCycle;
#endif
}

void Servo::setPosition90Deg(float degrees) { setPosition(degrees / 90); }

void Servo::setPosition120Deg(float degrees) { setPosition(degrees / 120); }

void Servo::setPosition180Deg(float degrees) { setPosition(degrees / 180); }

void Servo::setPosition360Deg(float degrees) { setPosition(degrees / 360); }

float Servo::getPosition()
{
#ifndef COMPILE_FOR_HOST
    float dutyCycle = pwm.getDutyCycle(pwmChannel);
#else
    float dutyCycle = this->dutyCycle;
#endif

    float pulse = dutyCycle * 1000000.0f / frequency;

    return (pulse - minPulse) / (maxPulse - minPulse);
}

float Servo::getPosition90Deg() { return getPosition() * 90; }

float Servo::getPosition180Deg() { return getPosition() * 1800; }

float Servo::getPosition360Deg() { return getPosition() * 3600; }

ServoData Servo::getState()
{
    return {TimestampTimer::getInstance().getTimestamp(),
            pwm.getTimer().getTimerNumber(), static_cast<uint8_t>(pwmChannel),
            getPosition()};
}

}  // namespace Boardcore
