/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Riccardo Sironi
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

#include "Servo.h"
#include "Valve.h"
namespace Boardcore
{
class ServoValve : public Valve
{
public:
    /**
     * @brief ValveSolenoid Constructor
     * @param pin Solenoid valve control pin
     */
    ServoValve(const ValveConfig& config, std::unique_ptr<Servo> servo)
        : Valve(config), servo(std::move(servo))
    {
    }
    ~ServoValve() {};

    // Move-only
    ServoValve(ServoValve&&)                 = default;
    ServoValve& operator=(ServoValve&&)      = default;
    ServoValve(const ServoValve&)            = delete;
    ServoValve& operator=(const ServoValve&) = delete;

    /**
     * @brief Sets the state of the solenoid valve (open/closed).
     * @param position position values greater than 0.5f are treated as high.
     */
    bool setPosition(float position) override
    {
        position *= config.limit;

        if (config.flipped)
            position = 1.0f - position;

        servo->setPosition(position, true);
        return true;
    };

    /**
     * @brief Returns the state of the solenoid valve (open/closed).
     * @returns True if open
     *
     * False if closed
     */
    float getPosition() override
    {
        float position = servo->getPosition();
        if (config.flipped)
            position = 1.0f - position;

        position /= config.limit;
        return position;
    };

    void enable() override { servo->enable(); }

    ValveType getType() const override { return ValveType::SERVO; }

    void backstep() override
    {
        switch (direction)
        {
            case Direction::OPEN:
                currentPosition -= Config::Servos::SERVO_BACKSTEP_AMOUNT;
                break;

            case Direction::CLOSE:
                currentPosition += Config::Servos::SERVO_BACKSTEP_AMOUNT;
                break;
        }

        // Clamp the position to the [0, 1] range
        currentPosition = std::min(1.0f, std::max(0.0f, currentPosition));

        setPosition(currentPosition);
    }

private:
    std::unique_ptr<Boardcore::Servo> servo;
};
}  // namespace Boardcore
