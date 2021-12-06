/* Copyright (c) 2017 Skyward Experimental Rocketry
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

#include <algorithm>

#include "Common.h"

class Timer : public Singleton<Timer>
{

    friend class Singleton<Timer>;

public:
    ~Timer();

    /**
     * Initialize the timer prescaler in order to have a counter tick equals,
     * in time terms, to the time needed to send 1.5 11 bit long characters at
     * the speed specified by @param refBaud
     */
    void init(uint32_t refBaud);

    /**
     * Start the timer
     */
    void start();

    /**
     * Stop the timer
     */
    void stop();

    /**
     * Set a new expiring setpoint @param ticks ahead from now for channel
     * specified by @param channel
     */
    void newSetpoint(uint8_t channel, uint8_t ticks);

    /**
     * @return true if the timer's @param channel expired, calling this
     * function will reset to fals the internal flag.
     */
    bool expired(uint8_t channel);

private:
    Timer();

    friend void IRQHandler();  ///< timer IRQ handler

    bool expFlags[2] = {false};
    float k;  ///< conversion factor internally used, see implementation

    Timer(const Timer& other) = delete;
    Timer& operator=(const Timer& other) = delete;
    bool operator==(const Timer& other)  = delete;
};
