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

#include <utils/DependencyManager/DependencyManager.h>

#include "ServoPCAValve.h"
#include "ServoValve.h"
#include "SolenoidValve.h"

namespace Boardcore
{

enum class ValveType
{
    SERVO,
    SOLENOID,
    SERVO_PCA
};

struct ValveConfig
{
    float limit                 = 1.0;    ///< Movement range limit
    bool flipped                = false;  ///< Whether the servo is flipped
    uint32_t defaultOpeningTime = 1000;   // Default opening time [ms]
    float defaultMaxAperture    = 1.0;    // Max aperture

    uint8_t openingEvent       = 0;  ///< Event to fire after opening
    uint8_t closingEvent       = 0;  ///< Event to fire after closing
    uint32_t openingTimeRegKey = 0;  ///< Registry key for opening
                                     ///< time
    uint32_t maxApertureRegKey = 0;  ///< Registry key for max
                                     ///< aperture
};
class Valve
{
public:
    virtual ~Valve() = default;

    // Move-only
    Valve(Valve&&)            = default;
    Valve& operator=(Valve&&) = default;

    // Delete copy
    Valve(const Valve&)            = delete;
    Valve& operator=(const Valve&) = delete;

    uint8_t getClosingEvent() const;
    uint8_t getOpeningEvent() const;

    uint32_t getOpeningTimeRegKey() const;
    uint32_t getMaxApertureRegKey() const;

    uint32_t getDefaultOpeningTime() const;
    float getDefaultMaxAperture() const;

    /**
     * @brief Returns the type of the valve
     *
     * @returns the ValveType
     */
    virtual ValveType getType() const = 0;

    virtual bool setPosition(float position) = 0;
    virtual float getPosition()              = 0;

    virtual void enable();
    virtual void backstep() = 0;

    float currentPosition = 0.0f;  ///< Current position in range [0, 1]

    enum class Direction
    {
        CLOSE,
        OPEN,
    } direction = Direction::CLOSE;  ///< Direction of the last valve move

protected:
    Valve(const ValveConfig& config) : config(config) {};
    ValveConfig config;  ///< Valve Config struct
};
}  // namespace Boardcore
