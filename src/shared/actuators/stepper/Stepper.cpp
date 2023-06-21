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
#include "Stepper.h"

namespace Boardcore
{

Stepper::Stepper(miosix::GpioPin stepPin, miosix::GpioPin directionPin,
                 float speed, float stepAngle, bool revertDirection,
                 uint16_t microStep, PinConfiguration pinConfiguration,
                 miosix::GpioPin enablePin)
    : stepPin(stepPin), directionPin(directionPin), speed(speed),
      stepAngle(stepAngle), revertDirection(revertDirection),
      microStep(microStep), pinConfig(pinConfiguration), enablePin(enablePin),
      currentDirection(Direction::CLOCKWISE)
{
    if (this->speed < 0)
        this->speed = 0;

    Stepper::setMicroStepping(microStep);

    // Start with the motor disabled
    disable();
}

void Stepper::enable()
{
    if (pinConfig == PinConfiguration::COMMON_CATHODE)
    {
        enablePin.low();
    }
    else
    {
        enablePin.high();
    }
    enabled = true;
}

void Stepper::disable()
{
    if (pinConfig == PinConfiguration::COMMON_CATHODE)
    {
        enablePin.high();
    }
    else
    {
        enablePin.low();
    }
    enabled = false;
}

void Stepper::setDirection()
{
    // Following the connections written in the stepper-driver datasheet for
    // moving the stepper clockwise we have that:
    // directionPin high:  stepper turns clockwise;
    // directionPin low: stepper turns counterclockwise;
    //
    // The revertDirection flag is used just for accounting for an inverted
    // polarity in the configuration.
    if (currentDirection == Direction::CLOCKWISE)
    {
        // To set the stepper-driver to turn CLOCKWISE we have to set the pin
        // high in common cathode mode and low in common anode mode (the
        // resulting potential difference should lead to a high logic value)
        if ((!revertDirection &&
             (pinConfig == PinConfiguration::COMMON_CATHODE)) ||
            (revertDirection && (pinConfig == PinConfiguration::COMMON_ANODE)))
        {
            directionPin.high();
        }
        else
        {
            directionPin.low();
        }
    }
    else
    {
        // To set the stepper-driver to turn COUNTER-CLOCKWISE we have to set
        // the pin low in common cathode mode and high in common anode mode (the
        // resulting potential difference should lead to a low logic value)
        if ((!revertDirection &&
             (pinConfig == PinConfiguration::COMMON_CATHODE)) ||
            (revertDirection && (pinConfig == PinConfiguration::COMMON_ANODE)))
        {
            directionPin.low();
        }
        else
        {
            directionPin.high();
        }
    }
}

void Stepper::move(int16_t steps)
{
    if (!enabled || speed == 0 || steps == 0)
        return;

    unsigned int halfStepDelay = 1e6 / (speed * 360 / stepAngle * microStep);
    int16_t stepsAbs;
    if (steps > 0)
    {
        // Go forward
        currentDirection = Direction::CLOCKWISE;
        setDirection();
        stepsAbs = steps;
    }
    else
    {
        // Go backwards
        currentDirection = Direction::COUNTER_CLOCKWISE;
        setDirection();
        stepsAbs = -steps;
    }

    miosix::delayUs(halfStepDelay);

    for (int i = 0; i < stepsAbs; i++)
    {
        if (pinConfig == PinConfiguration::COMMON_CATHODE)
        {
            stepPin.high();
        }
        else
        {
            stepPin.low();
        }

        miosix::delayUs(halfStepDelay);

        if (pinConfig == PinConfiguration::COMMON_CATHODE)
        {
            stepPin.low();
        }
        else
        {
            stepPin.high();
        }
        miosix::delayUs(halfStepDelay);
    }

    currentPositionDeg += steps * stepAngle / microStep;
}

bool Stepper::isEnabled() { return enabled; }

StepperData Stepper::getState(float moveDeg)
{
    return {TimestampTimer::getTimestamp(),
            static_cast<unsigned int>(stepPin.getPort()),
            stepPin.getNumber(),
            enabled,
            getCurrentDegPosition(),
            speed,
            moveDeg};
}

}  // namespace Boardcore