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

#pragma once

#include <miosix.h>
#include <functional>
#include <map>
#include <utility>

#include "ActiveObject.h"

using miosix::FastMutex;
using miosix::GpioPin;
using miosix::Lock;
using miosix::Thread;
using miosix::Unlock;

using std::function;
using std::map;
using std::pair;

/**
 * Class used to call a callback after a pin performs a specific transition
 * (RISING or FALLING edge) and stays in the new state for a specific amount of
 * time. Useful if you want to monitor pin transitions but you want to avoid
 * spurious state changes.
 *
 * A callback to monitor each state change no matter the thresold or the
 * transition is also available, in order to be able to observe the current
 * state of the pin
 */
class PinObserver : public ActiveObject
{
public:
    /**
     * @brief Pin transition
     * Actual enumaration value represents the stat of the pin after the
     * corresponding transition has occured.
     */
    enum class Transition : int
    {
        FALLING_EDGE = 0,
        RISING_EDGE  = 1
    };

    using OnStateChangeCallback =
        function<void(unsigned int, unsigned char, int)>;

    using OnTransitionCallback = function<void(unsigned int, unsigned char)>;

    /**
     * @brief Construct a new Pin Observer object
     *
     * @param poll_interval_ms Pin transition polling interval
     */
    PinObserver(unsigned int poll_interval_ms = 20)
        : poll_interval(poll_interval_ms)
    {
    }

    /**
     * Observe a pin for a specific transition, and optionally for every
     * single state change.
     *
     * The @param transition_cb function is called only if the two following
     * conditions are verified:
     * 1.  The transition specified in the @param transition is detected
     * 2.  The pin stays in the new state for at least detection_threshols
     * samples.
     *
     * The @param onstatechange_cb function [optional] is called at each state
     * change, both rising and falling edge, regardless of the @param
     * detection_threshold
     *
     * @param p GPIOA_BASE, GPIOB_BASE ...
     * @param n Which pin (0 to 15)
     * @param transition What transition to detect (RISING or FALLING edge)
     * @param transition_cb Function to call when the transition is detected and
     * the pin stays in the new configuration for at least @param
     * detection_threshold samples
     * @param detection_threshold How many times the pin should be observed in
     * the post-transition state to trigger the actual transition callback.
     * @param onstatechange_cb Function to be called at each pin state change,
     * no matter the threshold or the transition
     */
    void observePin(unsigned int p, unsigned char n, Transition transition,
                    OnTransitionCallback transition_cb,
                    unsigned int detection_threshold       = 1,
                    OnStateChangeCallback onstatechange_cb = nullptr)
    {
        Lock<FastMutex> lock(mtx_map);
        observed_pins.insert(std::make_pair(
            pair<unsigned int, unsigned char>({p, n}),
            ObserverData{GpioPin{p, n}, transition, transition_cb,
                         onstatechange_cb, detection_threshold}));
    }

    /**
     * @brief Stop monitoring the specified pin
     *
     * @param p GPIOA_BASE, GPIOB_BASE ...
     * @param n Which pin (0 to 15)
     */
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
                    pair<int, int> key     = it->first;
                    ObserverData& pin_data = it->second;

                    int old_state = pin_data.state;
                    int new_state = pin_data.pin.value();

                    // Save current state in the struct
                    pin_data.state = new_state;

                    // Are we in a post-transition state?
                    if (pin_data.state == static_cast<int>(pin_data.transition))
                    {
                        ++pin_data.detected_count;
                    }
                    else
                    {
                        pin_data.detected_count = 0;
                    }

                    // Pre-calcualate conditions in order to unlock the mutex
                    // only one time

                    bool state_change = pin_data.onstatechange_callback &&
                                        old_state != pin_data.state;
                    bool pin_triggered =
                        pin_data.detected_count == pin_data.detection_threshold;

                    {
                        Unlock<FastMutex> unlock(lock);

                        if (state_change)
                        {
                            pin_data.onstatechange_callback(
                                key.first, key.second, new_state);
                        }

                        if (pin_triggered)
                        {
                            pin_data.transition_callback(key.first, key.second);
                        }
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
        Transition transition;
        OnTransitionCallback transition_callback;
        OnStateChangeCallback onstatechange_callback;
        unsigned int detection_threshold;
        unsigned int detected_count;
        int state;  // 1 if HIGH, 0 if LOW

        ObserverData(GpioPin pin, Transition transition,
                     OnTransitionCallback transition_callback,
                     OnStateChangeCallback onstatechange_callback,
                     unsigned int detection_threshold)
            : pin(pin), transition(transition),
              transition_callback(transition_callback),
              onstatechange_callback(onstatechange_callback),
              detection_threshold(detection_threshold),
              // Set to this value to avoid detection if the pin is already in
              // the ending state of the "trigger" transition
              detected_count(detection_threshold + 1), state(0)
        {
        }
    };

    map<pair<unsigned int, unsigned char>, ObserverData> observed_pins;
    FastMutex mtx_map;

    unsigned int poll_interval;
    bool stopped = true;
};