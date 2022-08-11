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
                                      uint32_t detectionThreshold)
{
    // Try to insert the callback
    auto result = callbacks.insert(
        {pin, {callback, detectionThreshold, 0, 0, pin.value() == 1, 0}});

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
    const bool newState = pin.value();

    // Are we in a transition?
    if (pinData.lastState != newState)
    {
        count = 0;               // Yes, reset the counter
        pinData.changesCount++;  // And register the change
    }
    else
    {
        count++;  // No, continue to increment

        // If the count reaches the threshold, then trigger the event
        if (count > pinData.threshold)
        {
            if (newState)
                pinData.callback(PinTransition::RISING_EDGE);
            else
                pinData.callback(PinTransition::FALLING_EDGE);
        }
    }

    // Save the current pin status
    pinData.lastStateTimestamp = TimestampTimer::getTimestamp();
    pinData.lastState          = newState;
}

}  // namespace Boardcore
