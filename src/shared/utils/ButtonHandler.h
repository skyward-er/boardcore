/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <diagnostic/PrintLogger.h>

#include <functional>

#include "ActiveObject.h"

using miosix::Thread;

namespace Boardcore
{

static const int TICK_LENGTH           = 100;  // milliseconds
static const int LONG_PRESS_TICKS      = 10;
static const int VERY_LONG_PRESS_TICKS = 50;

enum class ButtonPress
{
    DOWN,      // Called as soon as the button is pressed
    UP,        // Called when the button is released
    SHORT,     // Short press
    LONG,      // Long press
    VERY_LONG  // Very long press
};

/**
 * @brief Class that detects if a button is pressed, long pressed or long-long
 * pressed and calls a callback in each case
 *
 * @tparam Button GPIO of the button
 */
template <typename Button>
class ButtonHandler : public ActiveObject
{
public:
    using ButtonCallback = std::function<void(uint8_t, ButtonPress)>;

    static bool isPressed() { return Button::value(); }

    ButtonHandler(uint8_t btnId, ButtonCallback callback)
        : btnId(btnId), callback(callback)
    {
        Button::mode(miosix::Mode::INPUT);
    }

    ~ButtonHandler() {}

protected:
    void run() override
    {
        while (!shouldStop() && callback)
        {
            if (Button::value())
            {
                if (!pressed && callback)
                {
                    callback(btnId, ButtonPress::DOWN);
                }

                pressedTicks++;
                pressed = true;
            }
            else if (pressed)  // if the button was unpressed since
                               // the last operation
            {
                if (pressedTicks >= VERY_LONG_PRESS_TICKS)
                {
                    LOG_DEBUG(logger, "Button pressed (very long) (%d ticks)",
                              pressedTicks);

                    callback(btnId, ButtonPress::VERY_LONG);
                }
                else if (pressedTicks >= LONG_PRESS_TICKS)
                {
                    LOG_DEBUG(logger, "Button pressed (long) (%d ticks)",
                              pressedTicks);
                    callback(btnId, ButtonPress::LONG);
                }
                else
                {
                    LOG_DEBUG(logger, "Button pressed (short) (%d ticks)",
                              pressedTicks);
                    callback(btnId, ButtonPress::SHORT);
                }

                callback(btnId, ButtonPress::UP);

                pressed      = false;
                pressedTicks = 0;
            }

            Thread::sleep(TICK_LENGTH);
        }
    }

private:
    uint8_t btnId    = 0;
    bool pressed     = false;
    int pressedTicks = 0;

    ButtonCallback callback;

    PrintLogger logger = Logging::getLogger("buttonhandler");
};

}  // namespace Boardcore
