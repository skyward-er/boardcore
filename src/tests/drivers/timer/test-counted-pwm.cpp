/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Alberto Nidasio
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

#include <drivers/timer/CountedPWM.h>
#include <miosix.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;

/**
 * This test shows how to use a CountedPWM to generate N pulses all done by
 * hardware!
 */

typedef Gpio<GPIOB_BASE, 4> tim3ch1;   // AF2
typedef Gpio<GPIOD_BASE, 12> tim4ch1;  // AF2

int main()
{
    tim3ch1::mode(Mode::ALTERNATE);
    tim3ch1::alternateFunction(2);

    tim4ch1::mode(Mode::ALTERNATE);
    tim4ch1::alternateFunction(2);

    CountedPWM pwm(
        TIM3, TimerUtils::Channel::CHANNEL_1, TimerUtils::TriggerSource::ITR3,
        TIM4, TimerUtils::Channel::CHANNEL_1, TimerUtils::TriggerSource::ITR2);
    pwm.setFrequency(2);
    pwm.setDutyCycle(0.5);

    std::thread th(
        [&]()
        {
            while (true)
            {
                printf("[%.2f] Counter: %d\tTarget: %ld\tIs generating: %d\n",
                       getTime() / 1e6 / 1000.0, pwm.getCurrentCount(),
                       TIM4->CCR1, pwm.isGenerating());
                Thread::sleep(250);
            }
        });
    th.detach();

    Thread::sleep(1000);

    pwm.generatePulses(4);
    Thread::sleep(1000);
    pwm.generatePulses(6);
    Thread::sleep(1000);
    pwm.generatePulses(3);

    Thread::sleep(5 * 1000);

    pwm.generatePulses(4);
    Thread::sleep(1000);
    pwm.updateTargetCount(6);

    Thread::sleep(5 * 1000);

    pwm.generatePulses(6);
    while (pwm.getCurrentCount() < 3)
        ;
    pwm.stop();

    pwm.setFrequency(10);
    while (true)
    {
        pwm.generatePulses(8);
        Thread::sleep(2500);
    }
}
