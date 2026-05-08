/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

#include <chrono>
#include <functional>

namespace Boardcore
{

/**
 * @brief Pin transition.
 */
enum class PinTransition : uint8_t
{
    FALLING_EDGE = 0,  ///< The pin goes from high to low.
    RISING_EDGE        ///< The pin goes from low to high.
};

/**
 * @brief Pin information.
 */
struct PinData
{
    // Number of periods the value was the same.
    uint32_t periodCount;
    // Last time the pin transitioned to a different state
    std::chrono::steady_clock::time_point lastTransitionTs;
    // Time when the last state change was detected
    std::chrono::steady_clock::time_point lastStateChangeTs;
    bool lastState;         ///< The last detected pin state
    uint32_t changesCount;  ///< Incremental count of the pin changes

    std::chrono::milliseconds getLastDetectionDelay() const
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            lastStateChangeTs - lastTransitionTs);
    }
};

/**
 * @brief Callback function type for pin transitions.
 *
 * @param transition The type of transition that triggered the callback.
 * @param data The data associated with the pin, updated with the latest
 * transition information.
 */
using PinCallback =
    std::function<void(PinTransition transition, const PinData& data)>;

/**
 * @brief Pin configuration.
 */
struct PinConfig
{
    PinCallback callback;  ///< The callback function.
    uint32_t threshold;    ///< Number of periods to trigger an event.
    bool reverted;         ///< Whether to revert the pin state.
};

}  // namespace Boardcore
