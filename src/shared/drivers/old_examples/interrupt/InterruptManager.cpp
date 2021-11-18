/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Author: Matteo Piazzolla
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

#include "InterruptManager.h"
#include "miosix.h"
#include <e20/e20.h>
#include <iostream>

miosix::FixedEventQueue<5> eq;

void* thread_eq_interrupt_manager(void*)
{
    eq.run();
    return NULL;
}

InterruptManager::InterruptManager() : interrupts{NULL}
{
    pthread_t t;
    pthread_create(&t,NULL,thread_eq_interrupt_manager,NULL);
}

void InterruptManager::OnInterruptEvent(unsigned n)
{
    IGenericInterrupt* const ptr = InterruptManager::GetInstance()->interrupts[n];
    if(ptr)
        ptr->OnReciveInt();
}

void __attribute__((naked)) EXTI0_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI0_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI0_IRQHandlerImpl()
{
    EXTI->PR = EXTI_PR_PR0;    
    if(eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 0))==false);
}

void __attribute__((naked)) EXTI1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI1_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI1_IRQHandlerImpl()
{
    EXTI->PR = EXTI_PR_PR1;
    if(eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 1))==false);
}

void __attribute__((naked)) EXTI2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI2_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI2_IRQHandlerImpl()
{
    EXTI->PR = EXTI_PR_PR2;    
    if(eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 2))==false);
}

void __attribute__((naked)) EXTI3_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI3_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI3_IRQHandlerImpl()
{
    EXTI->PR = EXTI_PR_PR3;    
    if(eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 3))==false);
}

void __attribute__((naked)) EXTI4_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI4_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI4_IRQHandlerImpl()
{
    EXTI->PR = EXTI_PR_PR4;
    if(eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 4))==false);
}

void __attribute__((naked)) EXTI9_5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22EXTI9_5_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI9_5_IRQHandlerImpl()
{
    const uint32_t status = EXTI->PR;
    EXTI->PR = status;
    if(status & GetPendingBitForLine(5) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 5))==false)
    { }
    if(status & GetPendingBitForLine(6) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 6))==false)
    { }
    if(status & GetPendingBitForLine(7) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 7))==false)
    { }
    if(status & GetPendingBitForLine(8) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 8))==false)
    { }
    if(status & GetPendingBitForLine(9) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 9))==false)
    { }
}

void __attribute__((naked)) EXTI15_10_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z24EXTI15_10_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI15_10_IRQHandlerImpl()
{
    const uint32_t status = EXTI->PR;
    EXTI->PR = status;
    if(status & GetPendingBitForLine(10) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 10))==false)
    { }
    if(status & GetPendingBitForLine(11) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 11))==false)
    { }
    if(status & GetPendingBitForLine(12) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 12))==false)
    { }
    if(status & GetPendingBitForLine(13) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 13))==false)
    { }
    if(status & GetPendingBitForLine(14) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 14))==false)
    { }
    if(status & GetPendingBitForLine(15) && eq.IRQpost(std::tr1::bind(InterruptManager::OnInterruptEvent, 15))==false)
    { }
}