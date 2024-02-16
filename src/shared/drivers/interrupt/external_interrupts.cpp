/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Luca Erbetta, Davide Mor
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

#include "external_interrupts.h"

#include <miosix.h>

#include "kernel/logging.h"

using namespace miosix;

/**
 * Since in the stm32 interrupts greater than 5 are handled in groups
 * (from 5 to 9 and from 10 to 15), this class provides a way to address single
 * interrupts in this range.
 *
 * In particular, an EXTIxx_IRQHandlerImpl() function is declared for every
 * interrupt from 5 to 15, and defined in a similar fashion as miosix's
 * 'stage_1_boot', i.e. handlers are "weak" and, if not overridden, will be
 * assigned to a default handler.
 *
 * NOTE: you will still have to enable the corresponding interrupt if you
 * want to use it, defining an IRQHandlerImpl is not enough.
 *
 * USAGE EXAMPLE:

    // Handle External Interrupt 9
    void __attribute__((used)) EXTI9_IRQHandlerImpl()
    {
        // Do some small operation, e.g. set a boolean flag
        // No need to clear interrupt
        // No need to use naked function
    }

    int main()
    {
        // 1. Set SYSCFG and EXTI registers (IMR, FTSR/RTSR)
        ...

        // 2. NVIC settings
        NVIC_EnableIRQ(EXTI9_5_IRQn);
        NVIC_SetPriority(EXTI9_5_IRQn, choose_a_priority);
    }

*/

// All unused interrupts call this function.
extern void unexpectedInterrupt();
extern "C" void Default_EXTI_Handler()
{
    IRQerrorLog("\r\n***Unexpected Peripheral interrupt [Boardcore]\r\n");
}

/**
 * Declaration of a separate IRQHandler for each external
 * interrupt. If no further implementation is provided, the
 * Default_Handler will be called.
 */
void __attribute__((weak)) EXTI0_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI0\r\n");
}

void __attribute__((weak)) EXTI1_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI1\r\n");
}

void __attribute__((weak)) EXTI2_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI2\r\n");
}

void __attribute__((weak)) EXTI3_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI3\r\n");
}

void __attribute__((weak)) EXTI4_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI4\r\n");
}

void __attribute__((weak)) EXTI5_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI5\r\n");
}

void __attribute__((weak)) EXTI6_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI6\r\n");
}

void __attribute__((weak)) EXTI7_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI7\r\n");
}

void __attribute__((weak)) EXTI8_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI8\r\n");
}

void __attribute__((weak)) EXTI9_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI9\r\n");
}

void __attribute__((weak)) EXTI10_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI10\r\n");
}

void __attribute__((weak)) EXTI11_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI11\r\n");
}

void __attribute__((weak)) EXTI12_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI12\r\n");
}

void __attribute__((weak)) EXTI13_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI13\r\n");
}

void __attribute__((weak)) EXTI14_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI14\r\n");
}

void __attribute__((weak)) EXTI15_IRQHandlerImpl()
{
    IRQerrorLog("\r\nUnexpected Peripheral interrupt EXTI15\r\n");
}

/**
 * Implementation of the IRQHandler that is triggered when
 * external interrupt 0 is raised.
 */
void __attribute__((naked)) EXTI0_IRQHandler()
{
    saveContext();
    EXTI->PR = EXTI_PR_PR0;
    asm volatile("bl _Z20EXTI0_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * external interrupt 1 is raised.
 */
void __attribute__((naked)) EXTI1_IRQHandler()
{
    saveContext();
    EXTI->PR = EXTI_PR_PR1;
    asm volatile("bl _Z20EXTI1_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * external interrupt 2 is raised.
 */
void __attribute__((naked)) EXTI2_IRQHandler()
{
    saveContext();
    EXTI->PR = EXTI_PR_PR2;
    asm volatile("bl _Z20EXTI2_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * external interrupt 3 is raised.
 */
void __attribute__((naked)) EXTI3_IRQHandler()
{
    saveContext();
    EXTI->PR = EXTI_PR_PR3;
    asm volatile("bl _Z20EXTI3_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * external interrupt 4 is raised.
 */
void __attribute__((naked)) EXTI4_IRQHandler()
{
    saveContext();
    EXTI->PR = EXTI_PR_PR4;
    asm volatile("bl _Z20EXTI4_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * any external interrupt between 5 and 9 is raised.
 */
void __attribute__((naked)) EXTI9_5_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z22EXTI9_5_IRQHandlerImplv");
    restoreContext();
}

/**
 * Implementation of the IRQHandler that is triggered when
 * any external interrupt between 10 and 15 is raised.
 */
void __attribute__((naked)) EXTI15_10_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z24EXTI15_10_IRQHandlerImplv");
    restoreContext();
}

/**
 * Read from the PR register which interrupt is pending
 * and call the corresponding IRQHandler.
 * If no flag in the range covered by this IRQ is set,
 * call default handler.
 */
void __attribute__((used)) EXTI9_5_IRQHandlerImpl()
{
    if (EXTI->PR & EXTI_PR_PR5)
    {
        EXTI->PR = EXTI_PR_PR5;  // Clear pending flag
        EXTI5_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR6)
    {
        EXTI->PR = EXTI_PR_PR6;
        EXTI6_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR7)
    {
        EXTI->PR = EXTI_PR_PR7;
        EXTI7_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR8)
    {
        EXTI->PR = EXTI_PR_PR8;
        EXTI8_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR9)
    {
        EXTI->PR = EXTI_PR_PR9;
        EXTI9_IRQHandlerImpl();
    }
    else
    {
#ifndef FLIGHT
        Default_EXTI_Handler();
#endif
    }
}

void __attribute__((used)) EXTI15_10_IRQHandlerImpl()
{
    if (EXTI->PR & EXTI_PR_PR10)
    {
        EXTI->PR = EXTI_PR_PR10;  // Clear pending flag
        EXTI10_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR11)
    {
        EXTI->PR = EXTI_PR_PR11;
        EXTI11_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR12)
    {
        EXTI->PR = EXTI_PR_PR12;
        EXTI12_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR13)
    {
        EXTI->PR = EXTI_PR_PR13;
        EXTI13_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR14)
    {
        EXTI->PR = EXTI_PR_PR14;
        EXTI14_IRQHandlerImpl();
    }
    else if (EXTI->PR & EXTI_PR_PR15)
    {
        EXTI->PR = EXTI_PR_PR15;
        EXTI15_IRQHandlerImpl();
    }
    else
    {
#ifndef FLIGHT
        Default_EXTI_Handler();
#endif
    }
}

constexpr unsigned ConvertGPIO_BASEtoUnsigned(unsigned P)
{
    // clang-format off
    return  P == GPIOA_BASE? 0 :
            P == GPIOB_BASE? 1 :
            P == GPIOC_BASE? 2 :
            P == GPIOD_BASE? 3 :
            P == GPIOE_BASE? 4 :
            P == GPIOF_BASE? 5 :
            P == GPIOG_BASE? 6 :
            P == GPIOH_BASE? 7 :
            P == GPIOI_BASE? 8 :
#if (defined (STM32F427xx) || defined (STM32F437xx) || defined (STM32F429xx) || defined (STM32F439xx))
            P == GPIOJ_BASE? 9 :
            P == GPIOK_BASE? 10 :
#endif
            0;
    // clang-format on
}

constexpr unsigned GetEXTI_IRQn(unsigned N)
{
    // clang-format off

    return N==0? EXTI0_IRQn :
           N==1? EXTI1_IRQn :
           N==2? EXTI2_IRQn :
           N==3? EXTI3_IRQn :
           N==4? EXTI4_IRQn :
           (N>=5&&N<=9)? EXTI9_5_IRQn :
           EXTI15_10_IRQn;

    // clang-format on
}

constexpr unsigned GetEXTICR_register_value(unsigned P, unsigned N)
{
    return (ConvertGPIO_BASEtoUnsigned(P) << ((N % 4) * 4));
}

constexpr unsigned GetEXTICR_register_mask(unsigned P, unsigned N)
{
    return (0b1111 << ((N % 4) * 4));
}

void enableExternalInterrupt(unsigned int gpioPort, unsigned int gpioNum,
                             InterruptTrigger trigger, unsigned int priority)
{
    auto exticrRegValue = GetEXTICR_register_value(gpioPort, gpioNum);

    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
        SYSCFG->EXTICR[int(gpioNum / 4)] |= exticrRegValue;
    }

    EXTI->IMR |= 1 << gpioNum;

    if (trigger == InterruptTrigger::RISING_EDGE ||
        trigger == InterruptTrigger::RISING_FALLING_EDGE)
        EXTI->RTSR |= 1 << gpioNum;

    if (trigger == InterruptTrigger::FALLING_EDGE ||
        trigger == InterruptTrigger::RISING_FALLING_EDGE)
        EXTI->FTSR |= 1 << gpioNum;

    NVIC_EnableIRQ(static_cast<IRQn_Type>(GetEXTI_IRQn(gpioNum)));
    NVIC_SetPriority(static_cast<IRQn_Type>(GetEXTI_IRQn(gpioNum)), priority);
}

void disableExternalInterrupt(unsigned int gpioPort, unsigned int gpioNum)
{
    EXTI->RTSR &= ~(1 << gpioNum);
    EXTI->FTSR &= ~(1 << gpioNum);
    EXTI->IMR &= ~(1 << gpioNum);

    auto exticrRegMask = GetEXTICR_register_mask(gpioPort, gpioNum);

    {
        FastInterruptDisableLock dLock;

        SYSCFG->EXTICR[int(gpioNum / 4)] &= ~exticrRegMask;
    }
}

void changeInterruptTrigger(unsigned int gpioPort, unsigned int gpioNum,
                            InterruptTrigger trigger)
{
    switch (trigger)
    {
        case InterruptTrigger::RISING_EDGE:
            EXTI->RTSR |= 1 << gpioNum;
            EXTI->FTSR &= ~(1 << gpioNum);
            break;

        case InterruptTrigger::FALLING_EDGE:
            EXTI->RTSR &= ~(1 << gpioNum);
            EXTI->FTSR |= 1 << gpioNum;
            break;

        case InterruptTrigger::RISING_FALLING_EDGE:
            EXTI->RTSR |= 1 << gpioNum;
            EXTI->FTSR |= 1 << gpioNum;
            break;
    }
}