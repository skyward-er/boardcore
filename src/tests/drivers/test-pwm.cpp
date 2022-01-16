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

#include <drivers/timer/PWM.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

typedef Gpio<GPIOC_BASE, 8> ch2;       // TIM8 CH2
typedef Gpio<GPIOG_BASE, 2> timeunit;  // Signaling output

void sep()
{
    timeunit::high();
    delayMs(1);
    timeunit::low();
    delayMs(499);
}

int main()
{
    printf("Setting up pins...\n");
    {
        FastInterruptDisableLock dLock;

        ch2::mode(Mode::ALTERNATE);
        ch2::alternateFunction(3);

        timeunit::mode(Mode::OUTPUT);
        timeunit::low();
    }

    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;

    PWM pwm(TIM8);

    pwm.setDutyCycle(GP16bitTimer::Channel::CHANNEL_2, 0.9);
    pwm.enableChannel(GP16bitTimer::Channel::CHANNEL_2);
    sep();

    for (int i = 0; i < 10; i++)
    {

        pwm.setDutyCycle(GP16bitTimer::Channel::CHANNEL_2, 0.7);
        sep();

        pwm.setDutyCycle(GP16bitTimer::Channel::CHANNEL_2, 0.5);
        sep();

        pwm.setDutyCycle(GP16bitTimer::Channel::CHANNEL_2, 0.3);
        sep();
    }

    while (true)
        Thread::sleep(10000);
}
