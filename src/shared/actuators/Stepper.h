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

#pragma once

#include <interfaces-impl/gpio_impl.h>
#include <interfaces/delays.h>
#include <utils/TestUtils/MockGpioPin.h>

namespace Boardcore
{

class Stepper
{
public:
    enum class PinConfiguration
    {
        COMMON_ANODE,    ///< All + signals connected to Vdd (3v3)
        COMMON_CATHODE,  ///< All - signals connected to Gnd
    };

    /**
     * @brief Construct a new Stepper object.
     *
     * By default the direction pin is held low when moving forward.
     * If the given speed is negative the motor wont move.
     *
     * @param stepPin Pin connected to the step signal of the driver.
     * @param directionPin Pin connected to the direction signal of the driver.
     * @param speed Number of rotations per second.
     * @param stepAngle Angle covered by one motor step.
     * @param revertDirection Whether or not revert the direction signal.
     */
    Stepper(
        miosix::GpioPin stepPin, miosix::GpioPin directionPin, float speed = 1,
        float stepAngle = 1.8, bool revertDirection = false,
        uint16_t microStep                = 1,
        PinConfiguration pinConfiguration = PinConfiguration::COMMON_CATHODE,
        miosix::GpioPin enablePin         = MockGpioPin());

    void enable();

    void disable();

    /**
     * @brief Changes the stepper motor speed.
     *
     * @param speed Speed in rotation per second. [rev/s]
     */
    virtual void setSpeed(float speed);

    /**
     * @brief Set the motor driver micro stepping configuration.
     *
     * This method has no effect if micro steps are changed via physical
     * switches.
     */
    virtual void setMicroStepping(uint16_t microStep);

    /**
     * @brief Overrides the driver's internal position counter.
     */
    void zeroPosition(float degrees = 0);

    /**
     * @brief Move the stepper motor by the specified amount of steps.
     */
    virtual void move(int16_t steps);

    /**
     * @brief Move the stepper motor by the specified amount of degrees.
     */
    void moveDeg(float degrees);

    /**
     * @brief Set the position of the stepper motor.
     *
     * Note that this function uses delayUs, thus it blocks execution based on
     * the stepper motor speed and movement.
     */
    virtual void setPosition(int16_t steps);

    /**
     * @brief Set the position of the stepper motor.
     *
     * Note that this function uses delayUs, thus it blocks execution based on
     * the stepper motor speed and movement.
     */
    void setPositionDeg(float degrees);

    int16_t getCurrentPosition();

    /**
     * @brief Returns the current absolute position of the stepper in degrees
     * [deg].
     */
    virtual float getCurrentDegPosition();

protected:
    /**
     * @brief Sets the directionPin to the right value to go in the direction
     * stored in `currentDirection`.
     */
    void setDirection();

    enum Direction : int8_t
    {
        CLOCKWISE = 1,  // To move the stepper clockwise seeing it from the top
                        // of the stepper
        COUNTER_CLOCKWISE = -1,  // To move the stepper counter-clockwise seeing
                                 // it from the top of the stepper
    };

    // This class is not copyable!
    Stepper& operator=(const Stepper&) = delete;
    Stepper(const Stepper& p)          = delete;

    miosix::GpioPin stepPin;
    miosix::GpioPin directionPin;
    float speed;      // [rev/s]
    float stepAngle;  // [deg/step]
    bool revertDirection;
    uint16_t microStep;
    PinConfiguration pinConfig;
    miosix::GpioPin enablePin;

    Direction currentDirection;    // Direction of the stepper
    float currentPositionDeg = 0;  // Absolute position counter [degrees]
};

inline void Stepper::setSpeed(float speed) { this->speed = speed; }

inline void Stepper::setMicroStepping(uint16_t microStep)
{
    this->microStep = microStep;
}

inline void Stepper::zeroPosition(float degrees)
{
    currentPositionDeg = degrees;
}

inline void Stepper::moveDeg(float degrees)
{
    move(degrees * microStep / stepAngle);
}

inline void Stepper::setPosition(int16_t steps)
{
    setPositionDeg(steps * stepAngle / microStep);
}

inline void Stepper::setPositionDeg(float position)
{
    moveDeg(position - getCurrentDegPosition());
}

inline int16_t Stepper::getCurrentPosition()
{
    return getCurrentDegPosition() * microStep / stepAngle;
}

inline float Stepper::getCurrentDegPosition() { return currentPositionDeg; }

}  // namespace Boardcore