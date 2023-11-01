/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "interfaces/delays.h"

namespace miosix
{

void delayMs(unsigned int mseconds)
{
#ifndef __CODE_IN_XRAM

#ifndef SYSCLK_FREQ_120MHz
#error "Delays are uncalibrated for this clock frequency"
#endif
    const unsigned int count = 29999;

    for (unsigned int i = 0; i < mseconds; i++)
    {
        // This delay has been calibrated to take 1 millisecond
        // It is written in assembler to be independent on compiler optimization
        asm volatile(
            "           mov   r1, #0     \n"
            "___loop_m: cmp   r1, %0     \n"
            "           itt   lo         \n"
            "           addlo r1, r1, #1 \n"
            "           blo   ___loop_m  \n" ::"r"(count)
            : "r1");
    }

#else  //__CODE_IN_XRAM
#error "No delays"
#endif  //__CODE_IN_XRAM
}

void delayUs(unsigned int useconds)
{
#ifndef __CODE_IN_XRAM

    // This delay has been calibrated to take x microseconds
    // It is written in assembler to be independent on compiler optimization
    asm volatile(
        "           mov   r1, #30    \n"
        "           mul   r2, %0, r1 \n"
        "           mov   r1, #0     \n"
        "___loop_u: cmp   r1, r2     \n"
        "           itt   lo         \n"
        "           addlo r1, r1, #1 \n"
        "           blo   ___loop_u  \n" ::"r"(useconds)
        : "r1", "r2");

#else  //__CODE_IN_XRAM
#error "No delays"
#endif  //__CODE_IN_XRAM
}

}  // namespace miosix
