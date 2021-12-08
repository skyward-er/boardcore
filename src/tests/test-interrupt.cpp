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

#include <drivers/interrupt/external_interrupts.h>
#include <miosix.h>
#include <stdio.h>

using namespace miosix;

bool itr = false;

// The compiler may remove this function since it doesn't know that it is onlt
// called from an assembly instruction. Use attribute used to avoid this.
void __attribute__((used)) EXTI0_IRQHandlerImpl() { itr = true; }

typedef Gpio<GPIOA_BASE, 0> user_button;

int main()
{
    user_button::mode(Mode::INPUT);

    enableExternalInterrupt(GPIOA_BASE, 0, InterruptTrigger::RISING_EDGE, 15);

    unsigned int counter = 0;
    while (true)
    {
        if (itr)
        {
            ++counter;
            printf("%d Interrupt received!\n", counter);
            itr = false;
        }
        Thread::sleep(50);
    }
}