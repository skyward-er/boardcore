#include "Common.h"
#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"
#include "interfaces-impl/hwmapping.h"

using namespace miosix;
using namespace interfaces;
using namespace actuators;

typedef Gpio<GPIOD_BASE, 12> ch1;  // ch1
typedef Gpio<GPIOD_BASE, 13> ch2;  // ch2


int main()
{
    printf("Setting up pins...\n");
    {
        FastInterruptDisableLock dLock;

        ch1::mode(Mode::ALTERNATE);
        ch1::alternateFunction(2);

        ch2::mode(Mode::ALTERNATE);
        ch2::alternateFunction(2);
    }
    PWM::Timer t{
        TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

    PWM pwm{t, 150};

    pwm.enableChannel(PWM::Channel::CH1, 0.3);

    Thread::sleep(500);
    pwm.start();
    Thread::sleep(500);
    pwm.enableChannel(PWM::Channel::CH2, 0.7, PWM::Mode::MODE_2);
    Thread::sleep(500);
    pwm.setDutyCycle(PWM::Channel::CH1, 0.7);
    Thread::sleep(500);
    pwm.disableChannel(PWM::Channel::CH1);
    Thread::sleep(500);
    pwm.stop();
    Thread::sleep(500);
    pwm.start();
    Thread::sleep(500);
    pwm.stop();

    while (1)
    {
        printf("End\n");
        Thread::sleep(10000);
    }

    return 0;
}
