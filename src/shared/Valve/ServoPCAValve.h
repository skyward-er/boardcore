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

namespace RIGv2
{
class ValveServoPCA : public Valve
{
public:
    ValveServoPCA(const ValveConfig& config, Boardcore::PCA9685& pca,
                  Boardcore::PCA9685Utils::Channel channel)
        : Valve(config), pca(pca), channel(channel)
    {
    }

    bool setPosition(float position) override
    {
        position *= config.limit;

        if (config.flipped)
            position = 1.0f - position;

        lastPosition = position;
        if (!pca.setDutyCycle(channel, position))
        {
            // How can I handle this error?
            return pca.setDutyCycle(channel, position);
        };
    };

    float getPosition() override
    {
        float position = lastPosition;
        if (config.flipped)
            position = 1.0f - position;

        position /= config.limit;
        return position;
    };

    ValveType getType() const override { return ValveType::SERVO_PCA; }

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
    Boardcore::PCA9685& pca;
    Boardcore::PCA9685Utils::Channel channel;
    float lastPosition = 0.0f;
};
}  // namespace RIGv2
