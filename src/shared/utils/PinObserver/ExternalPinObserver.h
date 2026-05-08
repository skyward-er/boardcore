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

#include <drivers/MCP23S17/MCP23S17.h>
#include <scheduler/TaskScheduler.h>
#include <utils/GpioPinCompare.h>

#include <chrono>
#include <functional>
#include <map>

#include "PinObserverUtils.h"

namespace Boardcore
{

/**
 * Class used to call a callback after a pin performs a specific transition
 * (RISING or FALLING edge) and stays in the new state for a specific amount of
 * time. Useful if you want to monitor pin transitions but you want to avoid
 * spurious state changes.
 *
 * A callback to monitor each state change no matter the threshold or the
 * transition is also available, in order to be able to observe the current
 * state of the pin.
 */
class ExternalPinObserver
{
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    /**
     * @brief Construct a new ExternalPinObserver object.
     *
     * @param scheduler Scheduler to be used by this ExternalPinObserver.
     * @param expander MCP23S17 expander used to read the pin values.
     * @param pollInterval Pin transition polling interval, defaults to 20 [ms].
     */
    ExternalPinObserver(TaskScheduler& scheduler, MCP23S17* expander,
                        uint32_t pollInterval = 20)
        : scheduler{scheduler}, expander{expander}, pollInterval{pollInterval}
    {
    }

    /**
     * Observe a pin for a specific transition, and optionally for every
     * single state change. The callback receives the transition and the time
     * point when the transition happened as parameters.
     *
     * @param pin Pin to listen to.
     * @param callback Function to call on pin events.
     * @param detectionThreshold How many times the pin should be observed in
     * the post-transition state to trigger the actual transition callback,
     * defaults to 1.
     * @return False if another callback was already registered for the pin.
     */
    bool registerPinCallback(ExternalGpioPin pin, PinCallback callback,
                             uint32_t detectionThreshold = 1,
                             bool reverted               = false);

    /**
     * @brief Returns the information for the specified pin.
     */
    PinData getPinData(ExternalGpioPin pin);

    /**
     * @brief Resets the changes counter for the specified pin.
     */
    void resetPinChangesCount(ExternalGpioPin pin);

private:
    /**
     * @brief This function is added to the scheduler for every pin registered
     * in the PinObserver.
     *
     * @param pin Pin whose value need to be checked.
     */
    void periodicPinValueCheck(ExternalGpioPin pin);

    TaskScheduler& scheduler;
    MCP23S17* expander;
    uint32_t pollInterval;

    /**
     * @brief Pin data used in the internal pin map.
     */
    struct PinEntry
    {
        PinConfig config;
        PinData data;
    };

    /// Map of all the callbacks registered in the PinObserver.
    std::map<ExternalGpioPin, PinEntry, GpioPinCompare> callbacks;
};

}  // namespace Boardcore
