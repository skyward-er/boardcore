/* Copyright (c) 2020-2021 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Davide Mor, Alberto Nidasio, Niccolò Betto
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

namespace Boardcore
{

/**
 * @brief Utility for microsecond timestamp values.
 *
 * @deprecated
 * This namespace provided access to sub-millisecond timestamp values for
 * precise timing requirements before Miosix 2.7, since at that time Miosix only
 * had millisecond-accurate tick values. It has now been deprecated in favor of
 * `miosix::getTime()` which has nanosecond resolution. The class is kept
 * for compatibility reasons and should be removed in the future.
 *
 * The old timer used a 32bit value and the TIM2 or TIM5 hardware peripherals,
 * at a frequency of 250KHz. This way the timer had a resolution of 4us, it
 * would overflow after 4.7 hours and only required a 2-bit shift left
 * operation to convert from counter tick to microseconds.
 */
namespace TimestampTimer
{
/**
 * @brief Returns the current timer value in microseconds.
 * @deprecated Use miosix::getTime() instead and update the code to nanoseconds.
 *
 * @return Current timestamp in microseconds.
 */
unsigned long long getTimestamp();
};  // namespace TimestampTimer
}  // namespace Boardcore
