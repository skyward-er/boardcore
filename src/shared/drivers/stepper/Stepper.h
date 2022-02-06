/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#include <miosix.h>

namespace Boardcore
{

class Stepper
{
public:
    /**
     * @brief Construct a new Stepper object.
     *
     * By default the direction pin is held low when moving forward.
     *
     * If the given speed is negative the motor wont move.
     *
     * @param stepPin Pin connected to the step signal of the driver.
     * @param directionPin Pin connected to the direction signal of the driver.
     * @param speed Number of rotations per second.
     * @param stepAngle Angle covered by one motor step.
     * @param revertDirection Whether or not revert the direction signal.
     */
    Stepper(miosix::GpioPin stepPin, miosix::GpioPin directionPin,
            float speed = 1, float stepAngle = 1.8,
            bool revertDirection = false);

    /**
     * @brief Changes the stepper motor speed.
     */
    void setSpeed(float speed);

    /**
     * @brief Move the stepper motor by the specified amount of steps.
     */
    void move(int32_t steps);

    /**
     * @brief Move the stepper motor by the specified amount of degrees.
     */
    void moveDeg(float degrees);

    /**
     * @brief Set the position of the stepper motor.
     *
     * Note that this function uses delayUs, thus it blocks execution based on
     * the stepper motor speed and movement.
     *
     * @param degrees Position in steps.
     */
    void setPosition(int32_t steps);

    /**
     * @brief Set the position of the stepper motor.
     *
     * Note that this function uses delayUs, thus it blocks execution based on
     * the stepper motor speed and movement.
     *
     * @param degrees Position in degrees.
     */
    void setPositionDeg(float degrees);

    uint32_t getCurrentPosition();

    float getCurrentDegPosition();

private:
    miosix::GpioPin stepPin;
    miosix::GpioPin directionPin;
    float speed;
    float stepAngle;  ///< Degrees per step.
    bool revertDirection;
    int32_t currentPosition = 0;
};

inline Stepper::Stepper(miosix::GpioPin stepPin, miosix::GpioPin directionPin,
                        float speed, float stepAngle, bool revertDirection)
    : stepPin(stepPin), directionPin(directionPin), speed(speed),
      stepAngle(stepAngle), revertDirection(revertDirection)
{
    if (speed < 0)
        speed = 0;
}

inline void Stepper::setSpeed(float speed) { this->speed = speed; }

inline void Stepper::move(int32_t steps)
{
    if (speed == 0)
        return;

    unsigned int halfStepDelay = 1e6 / (speed * 360 / stepAngle);

    if (revertDirection == (steps >= 0))
        directionPin.low();
    else
        directionPin.high();
    miosix::delayUs(halfStepDelay);

    int32_t stepsAbs = steps > 0 ? steps : -steps;
    for (int i = 0; i < stepsAbs; i++)
    {
        stepPin.high();
        miosix::delayUs(halfStepDelay);
        stepPin.low();
        miosix::delayUs(halfStepDelay);

        currentPosition++;
    }
}

inline void Stepper::moveDeg(float degrees) { move(degrees / stepAngle); }

inline void Stepper::setPosition(int32_t steps)
{
    move(steps - currentPosition);
}

inline void Stepper::setPositionDeg(float position)
{
    setPosition(position / stepAngle);
}

uint32_t Stepper::getCurrentPosition() { return currentPosition; }

float Stepper::getCurrentDegPosition() { return currentPosition * stepAngle; }

}  // namespace Boardcore