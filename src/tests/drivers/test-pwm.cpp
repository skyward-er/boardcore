#include "Common.h"
#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"

using namespace miosix;

typedef Gpio<GPIOC_BASE, 8> ch2;      // ch1
// typedef Gpio<GPIOD_BASE, 13> ch2;      // ch2
typedef Gpio<GPIOG_BASE, 2> timeunit;  // ch2

void sep()
{
    timeunit::high();
    Thread::sleep(25);
    timeunit::low();
    Thread::sleep(100);
}

int main()
{
    printf("Setting up pins...\n");
    {
        FastInterruptDisableLock dLock;

        // ch1::mode(Mode::ALTERNATE);
        // ch1::alternateFunction(2);

        ch2::mode(Mode::ALTERNATE);
        ch2::alternateFunction(3);

        timeunit::mode(Mode::OUTPUT);
        timeunit::low();
    }

    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    TIM8->PSC = 1;
    TIM8->CNT = 0;
    TIM8->EGR |= TIM_EGR_UG;
    TIM8->CR1 = TIM_CR1_CEN;

    printf("%lu\n", TIM8->CNT);
    /*PWM::Timer t{
        TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

    for(;;)
    {
        PWM pwm{t, 150};
        sep();

        pwm.start();
        sep();

        pwm.enableChannel(PWMChannel::CH2, 0.3);
        sep();

        pwm.setDutyCycle(PWMChannel::CH2, 0.7);
        sep();

        pwm.stop();
        sep();
    }

    sep();*/
    while (1)
    {
        printf("End\n");
        Thread::sleep(10000);
    }

    return 0;
}
