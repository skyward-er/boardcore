/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include "Valve.h"

namespace RIGv2
{
class ValveSolenoid : public Valve
{
public:
    /**
     * @brief ValveSolenoid Constructor
     * @param pin Solenoid valve control pin
     */
    ValveSolenoid(const ValveConfig& config, miosix::GpioPin pin)
        : Valve(config), pin(pin) {};

    /**
     * @brief Sets the state of the solenoid valve (open/closed).
     * @param position position values greater than 0.5f are treated as high.
     */
    bool setPosition(float position) override
    {
        if (position < 0.5f)
        {
            ValveSolenoid::pin.low();

        }  // set PIN to low
        else
        {
            ValveSolenoid::pin.high();
        }  // set PIN to high
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
        float position = ValveSolenoid::pin.value();

        if (config.flipped)
            position = 1.0f - position;

        return position;
    };

    ValveType getType() const override { return ValveType::SOLENOID; }

    void backstep() override
    {
        // Do nothing
    }

private:
    miosix::GpioPin pin;
};
}  // namespace RIGv2
