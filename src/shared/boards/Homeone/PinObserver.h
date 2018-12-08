/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_BOARDS_HELITEST_PINOBSERVER_H
#define SRC_SHARED_BOARDS_HELITEST_PINOBSERVER_H

#include <miosix.h>
#include <functional>
#include <map>
#include <utility>

#include "ActiveObject.h"

using miosix::FastMutex;
using miosix::GpioPin;
using miosix::Lock;
using miosix::Thread;

using std::function;
using std::map;
using std::pair;

/**
 * @brief Class used to call a callback when a pin state change is
 *
 */
class PinObserver : public ActiveObject
{
public:
    enum class Trigger : int
    {
        FALLING_EDGE = 0,
        RISING_EDGE  = 1
    };

    using PinCallbackFn = function<void(unsigned int, unsigned char)>;

    /**
     * @brief Construct a new Pin Observer object
     *
     * @param poll_interval_ms Pin state polling interval
     */
    PinObserver(unsigned int poll_interval_ms = 20)
        : ActiveObject(), poll_interval(poll_interval_ms)
    {
    }

    /**
     * @brief Start observing a pin
     *
     * @param p GPIOA_BASE, GPIOB_BASE ...
     * @param n Which pin (0 to 15)
     * @param trigger When to trigger
     * @param callback Callback to call when triggered
     * @param detection_threshold How long the pin should stay low (if trigger =
     * FALLING_EDGE) or high (if trigger = RISING EDGE) before triggering
     */
    void observePin(unsigned int p, unsigned char n, Trigger trigger,
                    PinCallbackFn callback,
                    unsigned int detection_threshold = 1)
    {
        Lock<FastMutex> lock(mtx_map);
        observed_pins.insert(
            std::make_pair(pair<unsigned int, char>({p, n}),
                           ObserverData{GpioPin{p, n}, trigger, callback,
                                        detection_threshold}));
    }

    void removeObservedPin(unsigned int p, unsigned char n)
    {
        Lock<FastMutex> lock(mtx_map);
        observed_pins.erase({p, n});
    }

protected:
    void run()
    {
        while (true)
        {
            {
                Lock<FastMutex> lock(mtx_map);

                for (auto it = observed_pins.begin(); it != observed_pins.end();
                     it++)
                {
                    pair<int, int> key  = it->first;
                    ObserverData& value = it->second;

                    if (value.pin.value() == static_cast<int>(value.trigger))
                    {
                        ++value.detected_count;
                    }
                    else
                    {
                        value.detected_count = 0;
                    }

                    if (value.detected_count == value.detection_threshold)
                    {
                        value.callback(key.first, key.second);
                    }
                }
            }
            Thread::sleep(poll_interval);
        }
    }

private:
    struct ObserverData
    {
        GpioPin pin;
        Trigger trigger;
        PinCallbackFn callback;
        unsigned int detection_threshold;
        unsigned int detected_count;

        ObserverData(GpioPin pin, Trigger trigger, PinCallbackFn callback,
                     unsigned int detection_threshold)
            : pin(pin), trigger(trigger), callback(callback),
              detection_threshold(detection_threshold), detected_count(0)
        {
        }
    };

    map<pair<unsigned int, unsigned char>, ObserverData> observed_pins;
    FastMutex mtx_map;

    unsigned int poll_interval;
    bool stopped = true;
};

#endif