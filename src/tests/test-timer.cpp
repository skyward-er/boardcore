/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include <cstdio>
#include "drivers/HardwareTimer.h"

using miosix::Thread;

int main()
{
    HardwareTimer<uint32_t, 2> timer2 = HardwareTimer<uint32_t, 2>::instance();
    HardwareTimer<uint16_t, 10> timer1 =
        HardwareTimer<uint16_t, 10>::instance();
/*
    timer1.setPrescaler(9999);
    timer1.start();
    timer2.start();
    int i = 0;
    for (;;)
    {
        uint32_t start2 = timer2.tick();
        Thread::sleep(10 * pow(10, i));
        uint32_t end2 = timer2.tick();

        uint16_t start1 = timer1.start();
        Thread::sleep(10 * pow(10, i));
        uint16_t end1 = timer1.tick();

        uint32_t t2 = end2 - start2;
        uint32_t t1 = end1 - start1;

        printf("Timer1: %.6f, %.3f, %.3f\n", timer1.toSeconds(t1),
               timer1.toMilliSeconds(t1), timer1.toMicroSeconds(t1));
        printf("Timer2: %.6f, %.3f, %.3f\n\n", timer2.toSeconds(t2),
               timer2.toMilliSeconds(t2), timer2.toMicroSeconds(t2));
        i++;
    }
    */
    return 0;
}