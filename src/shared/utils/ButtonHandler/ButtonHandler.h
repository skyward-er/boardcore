/* Copyright (c) 2015-2022 Skyward Experimental Rocketry
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

#pragma once

#include <Singleton.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <map>

namespace Boardcore
{

enum class ButtonEvent
{
    PRESSED,         // Called as soon as the button is pressed
    SHORT_PRESS,     // Called as soon as the button is released
    LONG_PRESS,      // Called as soon as the button is released
    VERY_LONG_PRESS  // Called as soon as the button is released
};

/**
 * @brief Comparison operator between GpioPins used for std::map.
 */
struct GpioPinCompare
{
    bool operator()(const miosix::GpioPin& lhs,
                    const miosix::GpioPin& rhs) const
    {
        if (lhs.getPort() == rhs.getPort())
            return lhs.getNumber() < rhs.getNumber();
        return lhs.getPort() < rhs.getPort();
    }
};

/**
 * @brief Utility to detects if buttons are pressed, long pressed or long-long
 * pressed and calls a callback in each case
 *
 * Note: The ButtonHandler assumes the all the buttons to be pulldown meaning
 * that when the button is pressed, the pin is assumed low.
 *
 * TODO: Allow to set pullup or pulldown configuration for each registered pin.
 */
class ButtonHandler : public Singleton<ButtonHandler>
{
    friend Singleton<ButtonHandler>;

    static constexpr uint32_t SAMPLE_PERIOD    = 100;  // 10Hz
    static constexpr int LONG_PRESS_TICKS      = 10;   // 1s
    static constexpr int VERY_LONG_PRESS_TICKS = 50;   // 5s

public:
    using ButtonCallback = std::function<void(ButtonEvent)>;

    /**
     * @brief Registers a callback on the specified pin.
     *
     * @param pin Pin to listen to.
     * @param callback Function to call on button events.
     * @return False if another callback was already registered for the pin.
     */
    bool registerButtonCallback(miosix::GpioPin pin, ButtonCallback callback);

    /**
     * @brief Unregisters the callback associated with the specified pin, if
     * any.
     *
     * @param pin Pin whose callback function is to be removed.
     * @return True if a callback was present and removed for the given pin.
     */
    bool unregisterButtonCallback(miosix::GpioPin pin);

    /**
     * @brief Starts the ButtonHandler's task scheduler.
     *
     * Note that the scheduler is started as soon as the ButtonHandler is first
     * used.
     *
     * @return Whether the task scheduler was started or not.
     */
    bool start();

    /**
     * @brief Stops the ButtonHandler's task scheduler.
     */
    void stop();

private:
    ButtonHandler();

    /**
     * @brief This function is added to the scheduler for every pin registered
     * in the ButtonHandler.
     *
     * @param pin Pin whose value need to be checked.
     */
    void periodicButtonValueCheck(miosix::GpioPin pin);

    TaskScheduler scheduler;

    /**
     * @brief Map of all the callbacks registered in the ButtonHandler.
     *
     * The key is the GpioPin for which the callback is registered. To used
     * GpioPin as a map key, the GpioPinCompare operator was defined as
     * explained here:
     * https://stackoverflow.com/questions/1102392/how-can-i-use-stdmaps-with-user-defined-types-as-key
     *
     * The type stored is a tuple containing:
     * - The button callback function;
     * - Whether or not the button was pressed in the last check iteration;
     * - The relative tick of the last pin value change.
     */
    std::map<miosix::GpioPin, std::tuple<ButtonCallback, bool, int>,
             GpioPinCompare>
        callbacks;
};

}  // namespace Boardcore