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

#include "Stepper.h"
#include "drivers/timer/CountedPWM.h"

namespace Boardcore
{

class StepperPWM : public Stepper
{
public:
    /**
     * @brief Construct a new StepperPWM object.
     *
     * By default the direction pin is held low when moving forward.
     * If the given speed is negative the motor wont move.
     *
     * @param pwm The CountedPWM object that will generate the N steps.
     * @param stepPin Pin connected to the step signal of the driver.
     * @param directionPin Pin connected to the direction signal of the driver.
     * @param speed Number of rotations per second.
     * @param stepAngle Angle covered by one motor step.
     * @param revertDirection Whether or not revert the direction signal.
     */
    StepperPWM(
        CountedPWM &pwm, miosix::GpioPin stepPin, miosix::GpioPin directionPin,
        float speed = 1, float stepAngle = 1.8, bool revertDirection = false,
        uint16_t microStep                = 1,
        PinConfiguration pinConfiguration = PinConfiguration::COMMON_CATHODE,
        miosix::GpioPin enablePin         = MockGpioPin());

    /**
     * @brief Changes the stepper motor speed.
     *
     * @param speed Speed in rotation per second. [rev/s]
     */
    void setSpeed(float speed) override;

    /**
     * @brief Set the motor driver micro stepping configuration.
     *
     * If micro steps are changed via physical switches we have to call this
     * method with the correct configuration to keep consistency in the speed of
     * the stepper.
     * @param microStep The micro steps that the stepper performs.
     */
    void setMicroStepping(uint16_t microStep) override;

    /**
     * @brief Move the stepper motor by the specified amount of steps.
     */
    void move(int16_t steps) override;

    /**
     * @brief Returns the current angle of the stepper.
     *
     * To calculate the correct angle of the stepper we add the angle reached in
     * the last actuation (saved in the variable `currentPositionDeg`) with the
     * conversion in angle of the actual stepper position (so, the number of
     * pulses actuated by the timer till now, that corresponds to the counter
     * register of the CounterTimer).
     *
     * Notice that on each actuation (calling one of the move or setPosition
     * methods) we save the angle reached using this method.
     */
    float getCurrentDegPosition() override;

private:
    // This class is not copyable!
    StepperPWM &operator=(const StepperPWM &) = delete;
    StepperPWM(const StepperPWM &p)           = delete;

    void setDirection();

    CountedPWM &pwm;
};

}  // namespace Boardcore