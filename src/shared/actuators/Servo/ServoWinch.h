/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

#ifndef COMPILE_FOR_HOST
#include <drivers/timer/PWM.h>
#endif

#include <units/Frequency.h>
#include <units/Time.h>

#include "ServoData.h"

#pragma once

namespace Boardcore
{

using namespace Units::Time;
using namespace Units::Frequency;
class ServoWinch
{
public:
#ifndef COMPILE_FOR_HOST
    /**
     * @brief Prepare the timer and sets the PWM output to 50% (which means that
     * the servo does not move).
     *
     * Note that the timer peripheral's clock is enabled automatically when the
     * PWM object is created.
     *
     * @param timer Timer peripheral used for the PWM signal.
     * @param pwmChannel Timer's channel used for the PWM signal.
     * @param frequency Frequency of the PWM driving the H-bridge.
     * @param minPulse Minimum signal pulse in microseconds.
     * @param maxPulse Maximum signal pulse in microseconds.
     */
    explicit ServoWinch(TIM_TypeDef* const timer,
                        TimerUtils::Channel pwmChannel,
                        Microsecond minPulse = 1000_us,
                        Microsecond maxPulse = 2000_us,
                        Hertz frequency      = 50.0_hz);
#else
    explicit ServoWinch(Microsecond minPulse = 1000_us,
                        Microsecond maxPulse = 2000_us,
                        Hertz frequency      = 50.0_hz);
#endif

    /**
     * @brief Starts producing the PWM signal.
     */
    void enable();

    /**
     * @brief Stops producing the PWM signal.
     */
    void disable();

    /**
     * @brief Set the rotation velocity of the servomotor.
     *
     * @param position velocity in range [0, 1] where values less than .5 means
     * negative rotation velocity.
     */
    void setVelocity(float position, bool limited = true);

    /**
     * @brief Returns the current velocity of the servomotor.
     *
     * Note that this velocity isn't necessarily the real servo velocity because
     * no feedback exists. It is merely the value of the signal generated.
     *
     * @return Velocity in range [0, 1].
     */
    float getVelocity();

    /**
     * @brief Returns the current position and the current timestamp.
     */
    ServoWinchData getState();

private:
    // This class is not copyable!
    ServoWinch& operator=(const ServoWinch&) = delete;
    ServoWinch(const ServoWinch& s)          = delete;

#ifndef COMPILE_FOR_HOST
    PWM pwm;
    TimerUtils::Channel pwmChannel;
#else
    float dutyCycle;
#endif

    Microsecond minPulse;
    Microsecond maxPulse;
    Hertz frequency;
};

}  // namespace Boardcore
