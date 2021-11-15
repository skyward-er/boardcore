/* Copyright (c) 2016-2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include <Common.h>
#include <Singleton.h>
#include <functional>

class WatchdogTimer : Singleton<WatchdogTimer>
{
    friend class Singleton<WatchdogTimer>;

public:
    /**
     * Start the timer
     */
    void start();

    /**
     * Stop the timer
     */
    void stop();

    /**
     * Clear timer's internal counter
     */
    void clear();

    /**
     * Set timeout period: callback after @param duration milliseconds after
     * counter is started
     */
    void setDuration(uint16_t duration);

    /**
     * Set a function that will be called when timer expires.
     * WARNING: the callback will be executed inside an interrupt routine, so
     * it must be written properly!! It cannot malloc, printf and other things
     * like that
     */
    void setCallback(std::function<void()> &callback) { irqCallback = callback; }

    /**
     * @return true if timer expired. Calling start() and clear() reset the
     * flag to false
     */
    bool expired() { return triggered; }

private:
    WatchdogTimer();
    ~WatchdogTimer();

    float ms_to_tick;  // conversion factor, number of counter ticks in a ms
    bool triggered;
    std::function<void()> irqCallback;

    friend void Irq_impl();

    WatchdogTimer(const WatchdogTimer& other);
    WatchdogTimer& operator=(const WatchdogTimer& other);
    bool operator==(const WatchdogTimer& other);
};
