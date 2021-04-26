/**
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

#include "servo.h"
#include <cmath>
#include <cstring>

// Initialize the pwm with 50 Hz frequency and 65535 levels of duty cycle
Servo::Servo(PWM::Timer t) : pwm(t, 50, 65535) { memset(&positions, 0, 4*sizeof(float)); }

Servo::~Servo() {}

void Servo::start() { pwm.start(); }

void Servo::stop() { pwm.stop(); }

void Servo::enable(PWMChannel channel)
{
    pwm.enableChannel(channel, calculateDutyCycle(positions[(int)channel]));
}

void Servo::disable(PWMChannel channel) { pwm.disableChannel(channel); }

void Servo::setPosition(PWMChannel channel, float pos)
{
    pos = fmax(0.0f, fmin(pos, 1.0f));

    positions[(int)channel] = pos;

    updateParameters();
}

float Servo::getPosition(PWMChannel channel) { return positions[(int)channel]; }

void Servo::setFrequency(unsigned int freq)
{
    frequency = fmax(10.0f, fmax(100.0f, freq));

    updateParameters();
}

void Servo::setMinPulseWidth(float min_pulse)
{
    this->min_pulse = fmax(500.0f, fmin(1300.0f, min_pulse));

    updateParameters();
}

void Servo::setMaxPulseWidth(float max_pulse)
{
    this->max_pulse = fmax(1700.0f, fmin(2500.0f, max_pulse));
    
    updateParameters();
}

void Servo::updateParameters()
{
    for (int i = 0; i < 4; i++)
    {
        pwm.setDutyCycle(static_cast<PWMChannel>(i),
                         calculateDutyCycle(positions[i]));
    }
}

float Servo::calculateDutyCycle(float position)
{
    float pulse = min_pulse + (max_pulse - min_pulse) * position;

    return pulse * frequency / 1000000.0f;
}
