/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <diagnostic/StackLogger.h>
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

namespace Boardcore
{

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
     * @param pollIntervalMs Pin transition polling interval
     */
    PinObserver(unsigned int pollIntervalMs = 20) : pollInterval(pollIntervalMs)
    {
    }

    /**
     * Observe a pin for a specific transition, and optionally for every
     * single state change.
     *
     * The @param transitionCb function is called only if the two following
     * conditions are verified:
     * 1.  The transition specified in the @param transition is detected
     * 2.  The pin stays in the new state for at least detection_threshols
     * samples.
     *
     * The @param onstatechangeCb function [optional] is called at each state
     * change, both rising and falling edge, regardless of the @param
     * detectionThreshold
     *
     * @param p GPIOA_BASE, GPIOB_BASE ...
     * @param n Which pin (0 to 15)
     * @param transition What transition to detect (RISING or FALLING edge)
     * @param transitionCb Function to call when the transition is detected and
     * the pin stays in the new configuration for at least @param
     * detectionThreshold samples
     * @param detectionThreshold How many times the pin should be observed in
     * the post-transition state to trigger the actual transition callback.
     * @param onstatechangeCb Function to be called at each pin state change,
     * no matter the threshold or the transition
     */
    void observePin(unsigned int p, unsigned char n, Transition transition,
                    OnTransitionCallback transitionCb,
                    unsigned int detectionThreshold       = 1,
                    OnStateChangeCallback onstatechangeCb = nullptr)
    {
        Lock<FastMutex> lock(mtxMap);
        observedPins.insert(
            std::make_pair(pair<unsigned int, unsigned char>({p, n}),
                           ObserverData{GpioPin{p, n}, transition, transitionCb,
                                        onstatechangeCb, detectionThreshold}));
    }

    /**
     * @brief Stop monitoring the specified pin
     *
     * @param p GPIOA_BASE, GPIOB_BASE ...
     * @param n Which pin (0 to 15)
     */
    void removeObservedPin(unsigned int p, unsigned char n)
    {
        Lock<FastMutex> lock(mtxMap);
        observedPins.erase({p, n});
    }

protected:
    void run()
    {
        while (true)
        {
            {
                Lock<FastMutex> lock(mtxMap);

                for (auto it = observedPins.begin(); it != observedPins.end();
                     it++)
                {
                    pair<int, int> key    = it->first;
                    ObserverData& pinData = it->second;

                    int oldState = pinData.state;
                    int newState = pinData.pin.value();

                    // Save current state in the struct
                    pinData.state = newState;

                    // Are we in a post-transition state?
                    if (pinData.state == static_cast<int>(pinData.transition))
                    {
                        ++pinData.detectedCount;
                    }
                    else
                    {
                        pinData.detectedCount = 0;
                    }

                    // Pre-calcualate conditions in order to unlock the mutex
                    // only one time

                    bool stateChange = pinData.onstatechangeCallback &&
                                       oldState != pinData.state;
                    bool pinTriggered =
                        pinData.detectedCount == pinData.detectionThreshold;

                    {
                        Unlock<FastMutex> unlock(lock);

                        if (stateChange)
                        {
                            pinData.onstatechangeCallback(key.first, key.second,
                                                          newState);
                        }

                        if (pinTriggered)
                        {
                            pinData.transitionCallback(key.first, key.second);
                        }
                    }
                }
            }

            StackLogger::getInstance().updateStack(THID_PIN_OBS);

            Thread::sleep(pollInterval);
        }
    }

private:
    struct ObserverData
    {
        GpioPin pin;
        Transition transition;
        OnTransitionCallback transitionCallback;
        OnStateChangeCallback onstatechangeCallback;
        unsigned int detectionThreshold;
        unsigned int detectedCount;
        int state;  // 1 if HIGH, 0 if LOW

        ObserverData(GpioPin pin, Transition transition,
                     OnTransitionCallback transitionCallback,
                     OnStateChangeCallback onstatechangeCallback,
                     unsigned int detectionThreshold)
            : pin(pin), transition(transition),
              transitionCallback(transitionCallback),
              onstatechangeCallback(onstatechangeCallback),
              detectionThreshold(detectionThreshold),
              // Set to this value to avoid detection if the pin is already in
              // the ending state of the "trigger" transition
              detectedCount(detectionThreshold + 1), state(0)
        {
        }
    };

    map<pair<unsigned int, unsigned char>, ObserverData> observedPins;
    FastMutex mtxMap;

    unsigned int pollInterval;
    bool stopped = true;
};

}  // namespace Boardcore
