/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <math.h>

#include "drivers/HardwareTimer.h"

namespace TimestampTimer
{

static const uint8_t TIMER_BITS      = 32;
static const uint8_t PRESCALER_VALUE = 127;
static const uint32_t RELOAD_VALUE   = pow(2, TIMER_BITS) - 1;  // 4294967295

extern HardwareTimer<uint32_t> timestamp_timer;

/**
 * @brief Enables and starts the 32-bits TIM2 timer peripheral, which can be
 * accessed via the timerstamp_timer pointer.
 */
void enableTimestampTimer();

/**
 * @return the current timestamp in microseconds
 */
inline uint64_t getTimestamp()
{
    return timestamp_timer.toIntMicroSeconds(timestamp_timer.tick());
}

}  // namespace TimestampTimer