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

#pragma weak EXTI5_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI6_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI7_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI8_IRQHandlerImpl = Default_EXTI_Handler
#pragma weak EXTI9_IRQHandlerImpl = Default_EXTI_Handler

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