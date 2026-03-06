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

namespace Boardcore
{
class SolenoidValve : public Valve
{
public:
    /**
     * @brief SolenoidValve Constructor
     * @param pin Solenoid valve control pin
     */
    SolenoidValve(const ValveConfig& config, miosix::GpioPin pin)
        : Valve(config), pin(pin) {};

    /**
     * @brief Does nothing to the solenoids, as they are enabled by default.
     */
    void enable() override {};

    /**
     * @brief Sets the state of the solenoid valve (open/closed).
     * @param position position values greater than 0.5f are treated as high.
     */
    bool setPosition(float position) override
    {
        if (position < 0.5f)
        {
            SolenoidValve::pin.low();

        }  // set PIN to low
        else
        {
            SolenoidValve::pin.high();
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
        float position = SolenoidValve::pin.value();

        if (config.flipped)
            position = 1.0f - position;

        return position;
    };

    /**
     * @brief Returns the type of the valve
     *
     * @returns the ValveType
     */
    ValveType getType() const override { return ValveType::SOLENOID; }

    /**
     * @brief  Does nothing on solenoids, as they can't possibly backstep.
     *
     */
    void backstep() override
    {
        // Do nothing
    }

private:
    miosix::GpioPin pin;
};
}  // namespace Boardcore
