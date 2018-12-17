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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include <stdio.h>

using namespace miosix;

bool itr = false;

// The compiler may remove this function since it doesn't know that it is onlt
// called from an assembly instruction. Use attribute used to avoid this.
void __attribute__((used)) EXTI0_IRQHandlerImpl()
{
    itr = true;
    EXTI->PR |= EXTI_PR_PR0;
}

// Attribute naked: Containing assembly code
void __attribute__((naked)) EXTI0_IRQHandler()
{
    saveContext();
    // bl _Z<length of function name><function name>v
    asm volatile("bl _Z20EXTI0_IRQHandlerImplv");
    restoreContext();
}

typedef Gpio<GPIOA_BASE, 0> user_button;

int main()
{
    user_button::mode(Mode::INPUT);
    
    {
        FastInterruptDisableLock l;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }
    // Refer to the datasheet for a detailed description on the procedure and interrupt registers

    // Clear the mask on the wanted line
    EXTI->IMR |= EXTI_IMR_MR0;

    // Trigger the interrupt on a falling edge
    EXTI->FTSR |= EXTI_FTSR_TR0;

    // Trigger the interrupt on a rising edge
    //EXTI->RTSR |= EXTI_RTSR_TR0;

    EXTI->PR |= EXTI_PR_PR0; // Reset pending register

    // Enable interrupt on PA0 in SYSCFG
    SYSCFG->EXTICR[0] = 0;

    // Enable the interrput in the interrupt controller
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI0_IRQn, 15);

    while (true)
    {
        if (itr)
        {
            printf("Interrupt received!\n");
            itr = false;
            ledOn();
            Thread::sleep(200);
            ledOff();
            Thread::sleep(200);
            ledOn();
            Thread::sleep(200);
            ledOff();
        }
    }
}