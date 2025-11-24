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

#include "ServoWinch.h"

#include <drivers/timer/TimestampTimer.h>

#include "miosix.h"

namespace Boardcore
{

#ifndef COMPILE_FOR_HOST

ServoWinch::ServoWinch(TIM_TypeDef* const timer, TimerUtils::Channel pwmChannel,
                       Microsecond minPulse, Microsecond maxPulse,
                       Hertz frequency)
    : pwm(timer, static_cast<uint16_t>(frequency.value())),
      pwmChannel(pwmChannel), minPulse(minPulse), maxPulse(maxPulse),
      frequency(frequency)
{
    setVelocity(0.5f);
}

void ServoWinch::enable() { pwm.enableChannel(pwmChannel); }

void ServoWinch::disable() { pwm.disableChannel(pwmChannel); }

#else

ServoWinch::ServoWinch(Microsecond minPulse, Microsecond maxPulse,
                       Hertz frequency)
    : minPulse(minPulse), maxPulse(maxPulse), frequency(frequency)
{
    setVelocity(0.5f);
}

void ServoWinch::enable() {}

void ServoWinch::disable() {}

#endif

void ServoWinch::setVelocity(float velocity, bool limited)
{
    if (limited)
    {
        if (velocity < 0)
            velocity = 0;
        else if (velocity > 1)
            velocity = 1;
    }

    Microsecond pulse = minPulse + (maxPulse - minPulse) * velocity;

    float dutyCycle = Second(pulse).value() * frequency.value();

#ifndef COMPILE_FOR_HOST
    pwm.setDutyCycle(pwmChannel, dutyCycle);
#else
    this->dutyCycle = dutyCycle;
#endif
}

float ServoWinch::getVelocity()
{
#ifndef COMPILE_FOR_HOST
    float dutyCycle = pwm.getDutyCycle(pwmChannel);
#else
    float dutyCycle = this->dutyCycle;
#endif

    Microsecond pulse = Microsecond(Second(dutyCycle / frequency.value()));

    return (pulse - minPulse).value() / (maxPulse - minPulse).value();
}

ServoWinchData ServoWinch::getState()
{
    return {TimestampTimer::getTimestamp(),

#ifndef COMPILE_FOR_HOST
            pwm.getTimer().getTimerNumber(), static_cast<uint8_t>(pwmChannel),
#else
            0, 0,
#endif
            getVelocity()};
}

}  // namespace Boardcore
