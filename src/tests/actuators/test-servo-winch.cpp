/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

#include <actuators/Servo/ServoWinch.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ClockUtils.h>

#include <iostream>

using namespace Boardcore;
using namespace miosix;

// Timer 4 CH2
GpioPin pin1(GPIOD_BASE, 13);

ServoWinch s1(TIM4, TimerUtils::Channel::CHANNEL_2);

int main()
{
    pin1.mode(Mode::ALTERNATE);
    pin1.alternateFunction(2);

    // Enable the timers
    s1.enable();

    // Control the fourth servo manually
    float percentage;

    printf("Please enter a percentage for the servo speed ");
    printf("or anything else to exit: ");

    while (std::cin >> percentage)
    {
        s1.setVelocity(percentage / 100);

        printf("Velocity set to: %.2f%%\n", percentage);

        printf("Please enter a percentage for the servo speed ");
        printf("or anything else to exit: ");
    }
}
