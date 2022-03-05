/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <interfaces-impl/gpio_impl.h>
#include <interfaces/delays.h>
#include <utils/testutils/MockGpioPin.h>

namespace Boardcore
{

class Stepper
{
public:
    enum class Microstep
    {
        MICROSTEP_1,
        MICROSTEP_2,
        MICROSTEP_4,
        MICROSTEP_8,
        MICROSTEP_16,
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
    Stepper(miosix::GpioPin stepPin, miosix::GpioPin directionPin,
            float speed = 1, float stepAngle = 1.8,
            bool revertDirection      = false,
            Microstep microstep       = Microstep::MICROSTEP_1,
            miosix::GpioPin enablePin = MockGpioPin(),
            miosix::GpioPin ms1Pin    = MockGpioPin(),
            miosix::GpioPin ms2Pin    = MockGpioPin(),
            miosix::GpioPin ms3Pin    = MockGpioPin());

    void enable();

    void disable();

    /**
     * @brief Changes the stepper motor speed.
     */
    void setSpeed(float speed);

    /**
     * @brief Set the motor driver microstepping configuration.
     */
    void setMicrostepping(Microstep microstep);

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
    int getMicrosteppingValue();

    miosix::GpioPin stepPin;
    miosix::GpioPin directionPin;
    float speed;
    float stepAngle;  ///< Degrees per step.
    bool revertDirection;
    Microstep microstep;
    miosix::GpioPin enablePin;
    miosix::GpioPin ms1Pin;
    miosix::GpioPin ms2Pin;
    miosix::GpioPin ms3Pin;

    int32_t currentPosition = 0;
};

inline Stepper::Stepper(miosix::GpioPin stepPin, miosix::GpioPin directionPin,
                        float speed, float stepAngle, bool revertDirection,
                        Microstep microstep, miosix::GpioPin enablePin,
                        miosix::GpioPin ms1Pin, miosix::GpioPin ms2Pin,
                        miosix::GpioPin ms3Pin)
    : stepPin(stepPin), directionPin(directionPin), speed(speed),
      stepAngle(stepAngle), revertDirection(revertDirection),
      microstep(microstep), enablePin(enablePin), ms1Pin(ms1Pin),
      ms2Pin(ms2Pin), ms3Pin(ms3Pin)
{
    if (speed < 0)
        speed = 0;

    setMicrostepping(microstep);

    // Start with the motor disabled
    disable();
}

inline void Stepper::enable() { enablePin.low(); }

inline void Stepper::disable() { enablePin.high(); }

inline void Stepper::setSpeed(float speed) { this->speed = speed; }

inline void Stepper::setMicrostepping(Microstep microstep)
{
    this->microstep = microstep;

    switch (microstep)
    {
        case Microstep::MICROSTEP_1:
            ms1Pin.low();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case Microstep::MICROSTEP_2:
            ms1Pin.high();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case Microstep::MICROSTEP_4:
            ms1Pin.low();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case Microstep::MICROSTEP_8:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case Microstep::MICROSTEP_16:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.high();
            break;
    }
}

inline void Stepper::move(int32_t steps)
{
    if (speed == 0)
        return;

    unsigned int halfStepDelay =
        1e6 / (speed * 360 / stepAngle * getMicrosteppingValue());

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
    }

    currentPosition += steps;
}

inline void Stepper::moveDeg(float degrees)
{
    move(degrees / stepAngle * getMicrosteppingValue());
}

inline void Stepper::setPosition(int32_t steps)
{
    move(steps - currentPosition);
}

inline void Stepper::setPositionDeg(float position)
{
    setPosition(position / stepAngle * getMicrosteppingValue());
}

inline uint32_t Stepper::getCurrentPosition() { return currentPosition; }

inline float Stepper::getCurrentDegPosition()
{
    return currentPosition * stepAngle;
}

inline int Stepper::getMicrosteppingValue()
{
    switch (microstep)
    {
        case Microstep::MICROSTEP_1:
            return 1;
        case Microstep::MICROSTEP_2:
            return 2;
        case Microstep::MICROSTEP_4:
            return 4;
        case Microstep::MICROSTEP_8:
            return 8;
        case Microstep::MICROSTEP_16:
            return 16;
    }

    return 1;
}

}  // namespace Boardcore