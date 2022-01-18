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

typedef Gpio<GPIOB_BASE, 4> ch1;
typedef Gpio<GPIOA_BASE, 7> ch2;
typedef Gpio<GPIOC_BASE, 8> ch3;

int main()
{
    ch1::mode(Mode::ALTERNATE);
    ch1::alternateFunction(2);
    ch2::mode(Mode::ALTERNATE);
    ch2::alternateFunction(2);
    ch3::mode(Mode::ALTERNATE);
    ch3::alternateFunction(2);

    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    PWM pwm(TIM3);

    pwm.enableChannel(TimerUtils::Channel::CHANNEL_1);
    pwm.enableChannel(TimerUtils::Channel::CHANNEL_2);
    pwm.enableChannel(TimerUtils::Channel::CHANNEL_3);

    float pos[] = {0.1, 0.5, 0.9};

    for (int i = 0;; i++)
    {
        pwm.setDutyCycle(TimerUtils::Channel::CHANNEL_1, pos[i % 3]);
        pwm.setDutyCycle(TimerUtils::Channel::CHANNEL_2, pos[(i + 1) % 3]);
        pwm.setDutyCycle(TimerUtils::Channel::CHANNEL_3, pos[(i + 2) % 3]);

        Thread::sleep(2000);
    }
}
