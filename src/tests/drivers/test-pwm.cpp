/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <drivers/timer/GeneralPurposeTimer.h>

#include "Common.h"
#include "drivers/pwm/pwm.h"

using namespace miosix;

// TODO: Update this test

typedef Gpio<GPIOC_BASE, 8> ch2;  // ch1
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
