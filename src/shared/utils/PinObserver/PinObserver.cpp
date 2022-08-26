/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <drivers/timer/TimestampTimer.h>

#include <functional>

namespace Boardcore
{

bool PinObserver::registerPinCallback(miosix::GpioPin pin, PinCallback callback,
                                      uint32_t detectionThreshold,
                                      bool reverted)
{
    // Try to insert the callback
    auto result = callbacks.insert({pin,
                                    {callback, detectionThreshold, 0, 0,
                                     pin.value() != reverted, 0, reverted}});

    // Check if the insertion took place
    if (result.second)
    {
        if (scheduler.addTask(
                std::bind(&PinObserver::periodicPinValueCheck, this, pin),
                SAMPLE_PERIOD, TaskScheduler::Policy::SKIP))
            return true;
        else
            callbacks.erase(pin);
    }

    return false;
}

bool PinObserver::start() { return scheduler.start(); }

void PinObserver::stop() { scheduler.stop(); }

bool PinObserver::isRunning() { return scheduler.isRunning(); }

PinData PinObserver::getPinData(miosix::GpioPin pin) { return callbacks[pin]; }

void PinObserver::resetPinChangesCount(miosix::GpioPin pin)
{
    callbacks[pin].changesCount = 0;
}

PinObserver::PinObserver() {}

void PinObserver::periodicPinValueCheck(miosix::GpioPin pin)
{
    // Make sure the pin informations are still present
    if (callbacks.find(pin) == callbacks.end())
        return;

    auto &pinData = callbacks[pin];

    // Retrieve the pin information
    uint32_t &count = pinData.periodCount;

    // Read the current pin status
    const bool newState = pin.value() != pinData.reverted;

    // Are we in a transition?
    if (pinData.lastState != newState)
    {
        // Register the current state change
        count++;

        // If the count reaches the threshold, then the change is confirmed and
        // we can trigger the event
        if (count > pinData.threshold)
        {
            count = 0;               // Reset the counter
            pinData.changesCount++;  // Register the change

            // Update the state
            pinData.lastStateTimestamp = TimestampTimer::getTimestamp();
            pinData.lastState          = newState;

            // Execute the callback
            pinData.callback(newState ? PinTransition::RISING_EDGE
                                      : PinTransition::FALLING_EDGE);
        }
    }
    else
    {
        // If the value is still the same as before, the counter is reset to 0
        count = 0;
    }
}

}  // namespace Boardcore
