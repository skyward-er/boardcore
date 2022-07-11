/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Author: Luca Erbetta, Alberto Nidasio
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

#include <functional>

namespace Boardcore
{

bool PinObserver::registerPinCallback(miosix::GpioPin pin, PinCallback callback,
                                      unsigned int detectionThreshold)
{
    // Try to insert the callback
    auto result =
        callbacks.insert({pin, {callback, detectionThreshold, pin.value(), 0}});

    // Check if the insertion took place
    if (result.second)
    {
        return scheduler.addTask(
            std::bind(&PinObserver::periodicPinValueCheck, this, pin),
            SAMPLE_PERIOD, TaskScheduler::Policy::SKIP);
    }

    return false;
}

bool PinObserver::unregisterPinCallback(miosix::GpioPin pin)
{
    return callbacks.erase(pin) != 0;
}

bool PinObserver::start() { return scheduler.start(); }

void PinObserver::stop() { scheduler.stop(); }

PinObserver::PinObserver() { scheduler.start(); }

void PinObserver::periodicPinValueCheck(miosix::GpioPin pin)
{
    // Make sure the pin informations are still present
    if (callbacks.find(pin) == callbacks.end())
        return;

    // Retrieve the pin information
    const PinCallback &callback           = std::get<0>(callbacks[pin]);
    const unsigned int detectionThreshold = std::get<1>(callbacks[pin]);
    bool &previousState                   = std::get<2>(callbacks[pin]);
    unsigned int &detectedCount           = std::get<3>(callbacks[pin]);

    // Read the current pin status
    const bool newState = pin.value();

    // Are we in a transition?
    if (previousState != newState)
    {
        detectedCount = 0;  // Yes, reset the counter
    }
    else
    {
        detectedCount++;  // No, continue to increment

        // If the count reaches the threshold, then trigger the event
        if (detectedCount > detectionThreshold)
        {
            if (newState)
                callback(PinTransition::RISING_EDGE);
            else
                callback(PinTransition::FALLING_EDGE);
        }
    }

    // Save the current pin status
    previousState = newState;
}

}  // namespace Boardcore
