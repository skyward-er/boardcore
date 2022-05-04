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

#ifndef COMPILE_FOR_HOST
#include <drivers/timer/PWM.h>
#endif

#pragma once

namespace Boardcore
{

/**
 * @brief Driver to operate a PWM controlled servo motor.
 *
 * Servo motors often are controlled via a PWM signal. A PWM signal is a simple
 * rectangular wave with a set frequency and duty cycle. Usually servo motor
 * listen for a pulse with a specific duration between two limits you can set to
 * this driver. The position is then proportional to the size of the pulse
 * relative to the range. A PWM signal is then used to periodically tell the
 * servo the position where it should be.
 *
 * Also Servo motors comes in a variety of motion range configurations. This
 * driver is prepared for non continuous rotation servo motor, which usually
 * have a range of 90, 180 or 360 degrees. The driver accepts the position
 * relative to the range (a percentage basically) but provides tree function to
 * convert to and from degrees.
 *
 * The PWM signal is disabled automatically when the Servo object is destructed
 * and the output of the signal pin will be low.
 *
 * Note that the peripheral clock of the undelying timer used to generate the
 * PWM signal, is enabled when the object is created and disabled when
 * destructed. When using the same timer for two or more sensors keep in mind
 * that you could encouter issues.
 */
class Servo
{
public:
#ifndef COMPILE_FOR_HOST
    /**
     * @brief Prepare the timer and sets the PWM output to the minimum.
     *
     * More specifically, the PWM output is prepared to be equal to minPulse but
     * it is not enabled! After creating the object the PWM signal is not
     * active. This is to ensure the servo motor doesn't move unexpectedly.
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
    explicit Servo(TIM_TypeDef* const timer, TimerUtils::Channel pwmChannel,
                   unsigned int minPulse = 1000, unsigned int maxPulse = 2000,
                   unsigned int frequency = 50);
    /**
     * @brief Prepare the timer and sets the PWM output to the minimum.
     *
     * @see Servo::Servo
     *
     * @param timer Timer peripheral used for the PWM signal.
     * @param pwmChannel Timer's channel used for the PWM signal.
     * @param frequency Frequency of the PWM driving the H-bridge.
     * @param minPulse Minimum signal pulse in microseconds.
     * @param maxPulse Maximum signal pulse in microseconds.
     * @param resetPulse Reset signal pulse in microseconds.
     */
    explicit Servo(TIM_TypeDef* const timer, TimerUtils::Channel pwmChannel,
                   unsigned int minPulse, unsigned int maxPulse,
                   unsigned int frequency, unsigned int resetPulse);
#else
    explicit Servo(unsigned int minPulse = 1000, unsigned int maxPulse = 2000,
                   unsigned int frequency = 50);
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
     * @brief Moves the servo to the reset position.
     */
    void reset();

    /**
     * @brief Set the position of the servomotor.
     *
     * @param position Position in range [0, 1].
     */
    void setPosition(float position);

    void setPosition90Deg(float degrees);

    void setPosition120Deg(float degrees);

    void setPosition180Deg(float degrees);

    void setPosition360Deg(float degrees);

    /**
     * @brief Returns the current position of the servomotor.
     *
     * Note that this position isn't necessarily the real servo position because
     * no feedback exists. It is merely the value of the signal generated.
     *
     * @return Position in range [0, 1].
     */
    float getPosition();

    float getPosition90Deg();

    float getPosition180Deg();

    float getPosition360Deg();

private:
    // This class is not copyable!
    Servo& operator=(const Servo&) = delete;
    Servo(const Servo& s)          = delete;

#ifndef COMPILE_FOR_HOST
    PWM pwm;
    TimerUtils::Channel pwmChannel;
#else
    float dutyCycle;
#endif

    float minPulse;
    float maxPulse;
    float resetPulse;
    float frequency;
};

}  // namespace Boardcore
