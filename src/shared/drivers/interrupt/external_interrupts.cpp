#include <miosix.h>

using namespace miosix;

/**
 * All unused interrupts call this function.
 */
extern void unexpectedInterrupt();

extern "C" void Default_EXTI_Handler() 
{
    unexpectedInterrupt();
}

/**
 * Declaration of a separate IRQHandler for each external
 * interrupt. If no further implementatio is provided, the
 * Default_Handler will be called.
 */
void __attribute__((weak))  EXTI5_IRQHandlerImpl();
void __attribute__((weak))  EXTI6_IRQHandlerImpl();
void __attribute__((weak))  EXTI7_IRQHandlerImpl();
void __attribute__((weak))  EXTI8_IRQHandlerImpl();
void __attribute__((weak))  EXTI9_IRQHandlerImpl();
void __attribute__((weak))  EXTI10_IRQHandlerImpl();
void __attribute__((weak))  EXTI11_IRQHandlerImpl();
void __attribute__((weak))  EXTI12_IRQHandlerImpl();
void __attribute__((weak))  EXTI13_IRQHandlerImpl();
void __attribute__((weak))  EXTI14_IRQHandlerImpl();
void __attribute__((weak))  EXTI15_IRQHandlerImpl();

#pragma weak EXTI5_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI6_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI7_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI8_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI9_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI10_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI11_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI12_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI13_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI14_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI15_IRQHandlerImpl = Default_EXTI_Handler

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
    if(EXTI->PR & EXTI_PR_PR5)
    {
        EXTI5_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR5;   // Clear pending flag
    }
    else if(EXTI->PR & EXTI_PR_PR6)
    {
        EXTI6_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR6;
    }
    else if(EXTI->PR & EXTI_PR_PR7)
    {
        EXTI7_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR7;
    }
    else if(EXTI->PR & EXTI_PR_PR8)
    {
        EXTI8_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR8;
    }
    else if(EXTI->PR & EXTI_PR_PR9)
    {
        EXTI9_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR9;
    }
    else
    {
        Default_EXTI_Handler();
    }
}

void __attribute__((used)) EXTI15_10_IRQHandlerImpl()
{
    if(EXTI->PR & EXTI_PR_PR10)
    {
        EXTI10_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR10;   // Clear pending flag
    }
    else if(EXTI->PR & EXTI_PR_PR11)
    {
        EXTI11_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR11;
    }
    else if(EXTI->PR & EXTI_PR_PR12)
    {
        EXTI12_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR12;
    }
    else if(EXTI->PR & EXTI_PR_PR13)
    {
        EXTI13_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR13;
    }
    else if(EXTI->PR & EXTI_PR_PR14)
    {
        EXTI14_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR14;
    }
    else if(EXTI->PR & EXTI_PR_PR15)
    {
        EXTI15_IRQHandlerImpl();
        EXTI->PR |= EXTI_PR_PR15;
    }
    else
    {
        Default_EXTI_Handler();
    }
}



/** USAGE EXAMPLE:

    // Handle External Interrupt 9
    void __attribute__((used)) EXTI9_IRQHandler()
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