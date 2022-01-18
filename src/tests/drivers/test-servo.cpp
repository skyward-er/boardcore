/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta, Alberto Nidasio
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

#include <drivers/servo/Servo.h>
#include <miosix.h>
#include <utils/ClockUtils.h>

using namespace Boardcore;
using namespace miosix;

using ps1 = Gpio<GPIOD_BASE, 12>;
using ps2 = Gpio<GPIOD_BASE, 13>;
using ps3 = Gpio<GPIOC_BASE, 7>;

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

    Servo s1(TIM4, TimerUtils::Channel::CHANNEL_1);
    Servo s2(TIM4, TimerUtils::Channel::CHANNEL_2);
    Servo s3(TIM8, TimerUtils::Channel::CHANNEL_2);

    s1.enable();
    s2.enable();
    s3.enable();

    float pos[] = {0, 0.5, 1.0};

    s1.setPosition(pos[0]);
    s2.setPosition(pos[1]);
    s3.setPosition(pos[2]);

    for (int i = 0;; i++)
    {
        // Cycle each servo between the 3 predefined positions
        s1.setPosition(pos[i % 3]);
        s2.setPosition(pos[(i + 1) % 3]);
        s3.setPosition(pos[(i + 2) % 3]);

        Thread::sleep(2000);
    }
}
