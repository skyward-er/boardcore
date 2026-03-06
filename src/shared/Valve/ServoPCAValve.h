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
#include "drivers/PCA9685/PCA9685.h"

namespace Boardcore
{
class ServoPCAValve : public Valve
{
public:
    ServoPCAValve(const ValveConfig& config, Boardcore::PCA9685& pca,
                  Boardcore::PCA9685Utils::Channel channel)
        : Valve(config), pca(pca), channel(channel)
    {
    }
    /**
     * @brief Does nothing when called on PCA controlled Valves
     */
    void enable() override {};

    /**
     * @brief Sets the position of the servo valve
     * @param position Where to, in the range from 0 to 1, set the servo
     * aperture
     */
    bool setPosition(float position) override
    {
        position *= config.limit;

        if (config.flipped)
            position = 1.0f - position;

        lastPosition = position;
        if (!pca.setDutyCycle(channel, position))
            return pca.setDutyCycle(channel, position);
        ;
    };

    /**
     * @brief Returns the last known position
     * @return The position
     */
    float getPosition() override
    {
        float position = lastPosition;
        if (config.flipped)
            position = 1.0f - position;

        position /= config.limit;
        return position;
    };
    /**
     * @brief Get the type of the valve ( Timer controlled servo, PCA controlled
     * servo, Solnoid )
     *
     * @returns the ValveType
     */
    ValveType getType() const override { return ValveType::SERVO_PCA; }

    /**
     * @brief Moves the valve via a small backstep.
     *
     */
    void backstep() override
    {
        switch (direction)
        {
            case Direction::OPEN:
                currentPosition -= SERVO_BACKSTEP_AMOUNT;
                break;

            case Direction::CLOSE:
                currentPosition += SERVO_BACKSTEP_AMOUNT;
                break;
        }

        // Clamp the position to the [0, 1] range
        currentPosition = std::min(1.0f, std::max(0.0f, currentPosition));

        setPosition(currentPosition);
    }

private:
    Boardcore::PCA9685& pca;
    Boardcore::PCA9685Utils::Channel channel;
    float lastPosition                       = 0.0f;
    static const float SERVO_BACKSTEP_AMOUNT = 0.02f;
};
}  // namespace Boardcore
