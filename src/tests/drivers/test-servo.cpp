/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <drivers/servo/servo.h>
#include <drivers/timer/TimerUtils.h>
#include <miosix.h>

using namespace Boardcore;
using namespace miosix;

using ps1 = Gpio<GPIOD_BASE, 12>;
using ps2 = Gpio<GPIOD_BASE, 13>;
using ps3 = Gpio<GPIOC_BASE, 7>;

PWMChannel s1_ch = PWMChannel::CH1;
PWMChannel s2_ch = PWMChannel::CH2;

PWMChannel s3_ch = PWMChannel::CH2;

int main()
{
    {
        FastInterruptDisableLock dLock;
        ps1::mode(Mode::ALTERNATE);
        ps2::mode(Mode::ALTERNATE);
        ps3::mode(Mode::ALTERNATE);

        ps1::alternateFunction(2);
        ps2::alternateFunction(2);

        ps3::alternateFunction(3);
    }

    PWM::Timer tim4{
        TIM4, &(RCC->APB1ENR), RCC_APB1ENR_TIM4EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

    PWM::Timer tim8{
        TIM8, &(RCC->APB2ENR), RCC_APB2ENR_TIM8EN,
        TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB2)};

    Servo s12{tim4};
    Servo s3{tim8};

    s12.start();
    s3.start();

    s12.enable(s1_ch);
    s12.enable(s2_ch);
    s3.enable(s3_ch);

    float pos[] = {0, 0.5f, 1.0f};

    s12.setPosition(s1_ch, pos[0]);
    s12.setPosition(s2_ch, pos[1]);
    s3.setPosition(s3_ch, pos[2]);
    for (;;)
        Thread::sleep(1000);
    int i = 0;
    for (;;)
    {
        // Cycle each servo between the 3 predefined positions
        s12.setPosition(s1_ch, pos[i % 3]);
        s12.setPosition(s2_ch, pos[(i + 1) % 3]);
        s3.setPosition(s3_ch, pos[(i + 2) % 3]);

        ++i;

        Thread::sleep(5000);
    }
}
