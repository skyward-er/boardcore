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

#pragma once

#include <Singleton.h>
#include <scheduler/TaskScheduler.h>
#include <utils/GpioPinCompare.h>

#include <map>

namespace Boardcore
{

/**
 * @brief Pin transition.
 */
enum class PinTransition
{
    FALLING_EDGE,  ///< The pin goes from high to low.
    RISING_EDGE    ///< The pin goes from low to high.
};

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
class PinObserver : public Singleton<PinObserver>
{
    friend Singleton<PinObserver>;

    static constexpr uint32_t SAMPLE_PERIOD = 100;  // 10Hz

public:
    using PinCallback = std::function<void(PinTransition)>;

    /**
     * Observe a pin for a specific transition, and optionally for every
     * single state change.
     *
     * @param pin Pin to listen to.
     * @param callback Function to call on button events.
     * @param detectionThreshold How many times the pin should be observed in
     * the post-transition state to trigger the actual transition callback,
     * defaults to 1.
     * @return False if another callback was already registered for the pin.
     */
    bool registerPinCallback(miosix::GpioPin pin, PinCallback callback,
                             unsigned int detectionThreshold = 1);

    /**
     * @brief Unregisters the callback associated with the specified pin, if
     * any.
     *
     * @param pin Pin whose callback function is to be removed.
     * @return True if a callback was present and removed for the given pin.
     */
    bool unregisterPinCallback(miosix::GpioPin pin);

    /**
     * @brief Starts the PinObserver's task scheduler.
     *
     * Note that the scheduler is started as soon as the PinObserver is first
     * used.
     *
     * @return Whether the task scheduler was started or not.
     */
    bool start();

    /**
     * @brief Stops the PinObserver's task scheduler.
     */
    void stop();

private:
    /**
     * @brief Construct a new PinObserver object.
     *
     * @param pollInterval Pin transition polling interval, defaults to 20 [ms].
     */
    PinObserver();

    /**
     * @brief This function is added to the scheduler for every pin registered
     * in the PinObserver.
     *
     * @param pin Pin whose value need to be checked.
     */
    void periodicPinValueCheck(miosix::GpioPin pin);

    TaskScheduler scheduler;

    /**
     * @brief Map of all the callbacks registered in the PinObserver.

     * The type stored is a tuple containing:
     * - The button callback function;
     * - Detection threshold: number of periods to trigger an event
     * - The last pin status;
     * - Number of periods the pin values stayed the same;
     */
    std::map<miosix::GpioPin,
             std::tuple<PinCallback, unsigned int, bool, unsigned int>,
             GpioPinCompare>
        callbacks;
};

}  // namespace Boardcore
