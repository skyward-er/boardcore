#include <drivers/HardwareTimer.h>
#include <Common.h>

int main()
{
    {
        miosix::FastInterruptDisableLock dLock;
        // Enable TIM2 + TIM3 + TIM4 peripheral clock
        RCC->APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN;
    }

    uint32_t prescaler = TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1);
    TRACE("Prescaler: %d\n", prescaler);

    HardwareTimer<uint32_t, TimerMode::Chain> timer1(TIM3, TIM4,
                                                    TimerTrigger::ITR2, 
                                                    prescaler);

    
    HardwareTimer<uint32_t, TimerMode::Single> timer2(TIM2, prescaler);

    timer1.start();
    timer2.start();

    while(true) {
        miosix::Thread::sleep(1000);

        uint32_t tick1 = timer1.tick();
        uint32_t tick2 = timer2.tick();

        TRACE("Timer1: %f\n", timer1.toMilliSeconds(tick1));
        TRACE("Timer2: %f\n", timer2.toMilliSeconds(tick1));

        // Delta should remain constant
        TRACE("Delta: %d\n", tick1 - tick2);
    }

    return 0;
}