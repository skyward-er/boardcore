#include "Common.h"
#include "drivers/HardwareTimer.h"
#include "drivers/pwm/pwm.h"
#include "interfaces-impl/hwmapping.h"

using namespace miosix;
using namespace interfaces;
using namespace actuators;

typedef Gpio<GPIOD_BASE, 12> ch1;      // ch1
typedef Gpio<GPIOD_BASE, 13> ch2;      // ch2
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

        ch1::mode(Mode::ALTERNATE);
        ch1::alternateFunction(2);

        ch2::mode(Mode::ALTERNATE);
        ch2::alternateFunction(2);

        timeunit::mode(Mode::OUTPUT);
        timeunit::low();
    }

    PWM::Timer t{
        TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

    {
        PWM pwm{t, 150};
        sep();

        pwm.start();
        sep();

        pwm.enableChannel(PWMChannel::CH1, 0.3);
        sep();

        pwm.setDutyCycle(PWMChannel::CH1, 0.7);
        sep();

        pwm.stop();
        sep();
    }
    sep();

    {
        PWM pwm{t, 500};
        sep();

        pwm.enableChannel(PWMChannel::CH1, 0.3);
        sep();

        pwm.enableChannel(PWMChannel::CH2, 0.3);
        sep();

        pwm.setDutyCycle(PWMChannel::CH2, 0.7);
        sep();

        pwm.start();
        sep();

        pwm.stop();
        sep();
    }
    sep();

    {
        PWM pwm{t, 500};
        pwm.enableChannel(PWMChannel::CH1, 0.3);
        pwm.enableChannel(PWMChannel::CH2, 0.3);
        pwm.start();
        sep();

        pwm.disableChannel(PWMChannel::CH1);
        sep();

        pwm.enableChannel(PWMChannel::CH1, 0.5);
        pwm.disableChannel(PWMChannel::CH2);
        sep();

        pwm.stop();
        sep();
    }
    sep();

    {
        PWM pwm{t, 500};
        pwm.enableChannel(PWMChannel::CH1, 0.3);
        pwm.enableChannel(PWMChannel::CH2, 0.1);
        pwm.start();
        sep();

        pwm.disableChannel(PWMChannel::CH1);
        pwm.disableChannel(PWMChannel::CH1);
        sep();

        pwm.enableChannel(PWMChannel::CH2, 0.9);
        sep();

        pwm.stop();
        sep();

        pwm.start();
        sep();
        pwm.stop();
    }
    sep();
    while (1)
    {
        printf("End\n");
        Thread::sleep(10000);
    }

    return 0;
}
