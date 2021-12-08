/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Davide Mor
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

#include <Debug.h>
#include <drivers/HardwareTimer.h>
#include <math.h>

namespace Boardcore
{

namespace TimestampTimer
{

/**
 * For timer resolution and duration refer to :
 * https://docs.google.com/spreadsheets/d/1B44bN6m2vnldQx9XVxZaBP8bDHqoLPREyHoaLh8s0UA/edit#gid=0
 *
 */
static const uint8_t PRESCALER_VALUE = 255;

#ifdef _ARCH_CORTEXM3_STM32
extern HardwareTimer<uint32_t, TimerMode::Chain> timestamp_timer;
#else
extern HardwareTimer<uint32_t, TimerMode::Single> timestamp_timer;
#endif

/**
 * @brief Enables and starts the timer peripheral, which can be
 * accessed via the timestamp_timer object.
 */
void enableTimestampTimer(uint8_t prescaler = PRESCALER_VALUE);

/**
 * @return the current timestamp in microseconds
 */
inline uint64_t getTimestamp()
{
    return timestamp_timer.toIntMicroSeconds(timestamp_timer.tick());
}

}  // namespace TimestampTimer

}  // namespace Boardcore
