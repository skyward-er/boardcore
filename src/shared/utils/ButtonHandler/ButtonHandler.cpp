/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "ButtonHandler.h"

#include <functional>

namespace Boardcore
{

bool ButtonHandler::registerButtonCallback(miosix::GpioPin pin,
                                           ButtonCallback callback)
{
    // Try to insert the callback
    auto result = callbacks.insert({pin, {callback, false, 0}});

    // Check if the insertion took place
    if (result.second)
    {
        return scheduler.addTask(
            std::bind(&ButtonHandler::periodicButtonValueCheck, this, pin),
            SAMPLE_PERIOD, TaskScheduler::Policy::SKIP);
    }

    return result.second;
}

bool ButtonHandler::start() { return scheduler.start(); }

void ButtonHandler::stop() { scheduler.stop(); }

ButtonHandler::ButtonHandler()
{
    // Start the scheduler immediately
    scheduler.start();
}

void ButtonHandler::periodicButtonValueCheck(miosix::GpioPin pin)
{
    // Make sure the pin informations are still present
    if (callbacks.find(pin) == callbacks.end())
        return;

    // Retrieve the pin information
    ButtonCallback &callback = std::get<0>(callbacks[pin]);
    bool &wasPressed         = std::get<1>(callbacks[pin]);
    int &pressedTicks        = std::get<2>(callbacks[pin]);

    // Read the current button status
    // Note: The button is assumed to be pressed if the pin value is low
    // (pulldown)
    bool isNowPressed = !pin.value();

    if (isNowPressed)
    {
        // If the pin was not pressed before it has just been pressed
        if (!wasPressed && callback)
            callback(ButtonEvent::PRESSED);

        // Increment the tick
        pressedTicks++;
    }
    // If the button was pressed before and just released
    else if (wasPressed)
    {
        if (pressedTicks >= VERY_LONG_PRESS_TICKS)
            callback(ButtonEvent::VERY_LONG_PRESS);
        else if (pressedTicks >= LONG_PRESS_TICKS)
            callback(ButtonEvent::LONG_PRESS);
        else
            callback(ButtonEvent::SHORT_PRESS);

        // Reset the ticks
        pressedTicks = 0;
    }

    // Save the current button status
    wasPressed = isNowPressed;
}

}  // namespace Boardcore