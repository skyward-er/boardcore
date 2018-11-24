#include "Common.h"
#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"

using namespace miosix;

typedef Gpio<GPIOD_BASE, 12> pwmOut;   // ch1
typedef Gpio<GPIOD_BASE, 13> pwmOut2;  // ch2

int main()
{
    {
        FastInterruptDisableLock dLock;

        pwmOut::alternateFunction(2);
        pwmOut::mode(Mode::ALTERNATE);
    }

    PWM::Timer t{
        TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

    PWM pwm{t};

    pwm.enable(0, 1);

    pwm.start();

    Thread::sleep(20000);
    pwm.setDutyCycle(0.1);
    Thread::sleep(20000);
    pwm.stop();
    pwm.disable();

    while (1)
    {
        printf("End\n");
        Thread::sleep(10000);
    }

    return 0;
}
