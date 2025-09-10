/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Niccolò Betto
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

#include "PinObserver.h"

namespace Boardcore
{

bool PinObserver::registerPinCallback(miosix::GpioPin pin, PinCallback callback,
                                      uint32_t detectionThreshold,
                                      bool reverted)
{
    // Try to insert the callback
    auto result =
        callbacks.insert({pin, PinEntry{
                                   .config =
                                       PinConfig{
                                           .callback  = std::move(callback),
                                           .threshold = detectionThreshold,
                                           .reverted  = reverted,
                                       },
                                   .data =
                                       PinData{
                                           .periodCount       = 0,
                                           .lastTransitionTs  = TimePoint{},
                                           .lastStateChangeTs = TimePoint{},
                                           // Broken on cppcheck 2.7
                                           // cppcheck-suppress internalAstError
                                           .lastState = pin.value() != reverted,
                                           .changesCount = 0,
                                       },
                               }});

    // Check if the insertion took place
    if (!result.second)
        return false;

    auto taskId =
        scheduler.addTask([this, pin] { periodicPinValueCheck(pin); },
                          pollInterval, TaskScheduler::Policy::RECOVER);

    // Check if the task was added successfully
    if (!taskId)
    {
        callbacks.erase(pin);
        return false;
    }

    return true;
}

PinData PinObserver::getPinData(miosix::GpioPin pin)
{
    auto pinEntry = callbacks.find(pin);
    if (pinEntry == callbacks.end())
        return PinData{};

    return pinEntry->second.data;
}

void PinObserver::resetPinChangesCount(miosix::GpioPin pin)
{
    auto pinEntry = callbacks.find(pin);
    if (pinEntry == callbacks.end())
        return;

    pinEntry->second.data.changesCount = 0;
}

void PinObserver::periodicPinValueCheck(miosix::GpioPin pin)
{
    auto pinEntry = callbacks.find(pin);

    // Make sure the pin informations are still present
    if (pinEntry == callbacks.end())
        return;

    // Retrieve the pin information
    auto& pinConfig = pinEntry->second.config;
    auto& pinData   = pinEntry->second.data;
    auto& count     = pinData.periodCount;

    // Read the current pin status
    const bool newState = pin.value() != pinConfig.reverted;

    // Are we in a transition?
    if (pinData.lastState != newState)
    {
        // Register the current state change
        count++;

        // If the count reaches the threshold, then the change is confirmed and
        // we can trigger the event
        if (count >= pinConfig.threshold)
        {
            count = 0;               // Reset the counter
            pinData.changesCount++;  // Register the change

            // Update the state
            pinData.lastStateChangeTs = Clock::now();
            pinData.lastState         = newState;

            // Execute the callback
            pinConfig.callback(newState ? PinTransition::RISING_EDGE
                                        : PinTransition::FALLING_EDGE,
                               pinData);
        }
    }
    else
    {
        // Update the last transition timestamp as the time it was still in the
        // previous state. Once a transition happens this won't be updated and
        // will contain the last time the pin was in the previous state
        pinData.lastTransitionTs = Clock::now();

        // If the value is still the same as before, the counter is reset to 0
        count = 0;
    }
}

}  // namespace Boardcore
