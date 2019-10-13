/**
 *
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/HardwareTimer.h>
#include <drivers/pwm/pwm.h>

#pragma once

/**
 * Class used to control servomotors. This impementation differs from the
 * one in miosix (in servo_stm32.h)  as it can control servos connected to any
 * set of pins & timers, but it is not thread safe.
 */
class Servo
{
public:
    Servo(PWM::Timer t);
    ~Servo();

    /**
     * Start producing the output PWM waveforms for the enabled channels
     * @return
     */
    void start();

    /**
     * Stop producing the output waveforms.
     * @return
     */
    void stop();

    /**
     * Enable the specified channel. The driver will start to output the PWM
     * waveform to the specified channel once start() is called;
     *
     * @param channel Which servo to enable
     */
    void enable(PWMChannel channel);

    /**
     * Disable the channel. No output PWM waveform is generated until the
     * channel is enabled again.
     *
     * @param channel Which servo to disable
     */
    void disable(PWMChannel channel);

    /**
     * Sets the position of the servomotor.
     * Accepts a value from 0 to 1.
     * @param channel Which servo to control
     * @param pos Position in range [0, 1]
     */
    void setPosition(PWMChannel channel, float pos);

    /**
     * Returns the current position of the specified servo
     * @param channel Which servo
     * @return Position of the selected servo
     */
    float getPosition(PWMChannel channel);

    /**
     * Sets the output PWM frequency. Default value is 50 Hz, must be between 10
     * Hz and 100 Hz
     * @param freq Frequency of the output PWM
     */
    void setFrequency(unsigned int freq);

    /**
     * Sets the minimum duration of the PWM pulse.
     * That is, how long the PWM waveform stays high when the position is 0.
     *
     * Must be between 500 and 1300. Default value is 1000.
     * @param min_pulse Pulse width in microseconds.
     */
    void setMinPulseWidth(float min_pulse);

    /**
     * Sets the mmaximum duration of the PWM pulse.
     * That is, how long the PWM waveform stays high when the position is 1.
     * Must be between 1700 and 2500. Default value is 2000.
     * @param min_pulse Pulse width in microseconds.
     */
    void setMaxPulseWidth(float max_pulse);

private:
    float calculateDutyCycle(float position);

    void updateParameters();

    // Deleted copy constructor
    Servo(const Servo& s) = delete;

    PWM pwm;

    float positions[4];

    float frequency = 50;
    float min_pulse = 1000;
    float max_pulse = 2000;
};