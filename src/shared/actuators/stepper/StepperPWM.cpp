/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Emilio Corigliano
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
#include "StepperPWM.h"

namespace Boardcore
{

StepperPWM::StepperPWM(CountedPWM &pwm, miosix::GpioPin stepPin,
                       miosix::GpioPin directionPin, float speed,
                       float stepAngle, bool revertDirection,
                       uint16_t microStep, PinConfiguration pinConfiguration,
                       miosix::GpioPin enablePin)
    : Stepper(stepPin, directionPin, speed, stepAngle, revertDirection,
              microStep, pinConfiguration, enablePin),
      pwm(pwm)
{
    if (this->speed < 0)
        this->speed = 0;

    Stepper::setDirection();
    StepperPWM::setMicroStepping(microStep);

    // Start with the motor disabled
    disable();
}

void StepperPWM::setSpeed(float speed)
{
    this->speed = speed;
    pwm.setFrequency(speed * 360 / stepAngle * microStep);
}

void StepperPWM::setMicroStepping(uint16_t microStep)
{
    // Resize the pulses to generate and the current pulses generated to use the
    // new microsteps (keeping constant the angle of movement)
    pwm.updateTargetCount(pwm.getTargetCount() * this->microStep / microStep);
    pwm.updateCurrentCount(pwm.getCurrentCount() * this->microStep / microStep);

    Stepper::setMicroStepping(microStep);
    pwm.setFrequency(speed * 360 / stepAngle * microStep);
}

void StepperPWM::move(int16_t steps)
{
    if (!enabled || speed == 0 || steps == 0)
        return;

    // First update currentPositionDeg. This method corrects the initial
    // position saved with the current state of the CountTimer counter register,
    // so that we account only for the pulses actually generated.
    currentPositionDeg = getCurrentDegPosition();

    if (steps > 0)
    {
        // Go forward
        currentDirection = Direction::CLOCKWISE;
        Stepper::setDirection();

        // Move
        pwm.generatePulses(steps);
    }
    else
    {
        // Go backwards
        currentDirection = Direction::COUNTER_CLOCKWISE;
        Stepper::setDirection();

        // Move
        pwm.generatePulses(-steps);
    }
}

float StepperPWM::getCurrentDegPosition()
{
    return currentPositionDeg +
           currentDirection * pwm.getCurrentCount() * stepAngle / microStep;
}

}  // namespace Boardcore