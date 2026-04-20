/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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
#include "drivers/MCP23S17/MCP23S17.h"

namespace Boardcore
{
class ExternalSolenoidValve : public Valve
{
public:
    /**
     * @brief ExternalSolenoidValve Constructor
     * @param setValue lambda function to set the state of the solenoid valve:
     *        true for open, false for closed
     *
     */
    ExternalSolenoidValve(const ValveConfig& config, const ExternalGpioPin& pin,
                          MCP23S17& expander)
        : Valve(config), externalPin(pin), expander(&expander) {};

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
        if (config.flipped)
            position = 1.0f - position;

        if (position < 0.5f)
        {
            expander->setPinValue(externalPin.getPort(), externalPin.getPin(),
                                  false);
            lastPosition = 0.0f;
        }  // set PIN to low
        else
        {
            expander->setPinValue(externalPin.getPort(), externalPin.getPin(),
                                  true);
            lastPosition = 1.0f;
        }  // set PIN to high
        return true;
    };

    /**
     * @brief Returns the state of the solenoid valve (open/closed).
     * @returns True if open, false if closed
     */
    float getPosition() override
    {
        float position = lastPosition;

        if (config.flipped)
            position = 1.0f - position;

        return position;
    };

    /**
     * @brief Returns the type of the valve
     *
     * @returns the ValveType
     */
    ValveType getType() const override { return ValveType::EXTERNAL_SOLENOID; }

    /**
     * @brief  Does nothing on solenoids, as they can't possibly backstep.
     *
     */
    void backstep() override
    {
        // Do nothing
    }

private:
    float lastPosition = 0.0f;
    ExternalGpioPin externalPin;
    MCP23S17* expander;  ///< Pointer to the MCP23S17 controlling the valve pin
};
}  // namespace Boardcore
