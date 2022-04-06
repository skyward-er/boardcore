/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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
#include <scheduler/TaskScheduler.h>
#include <utils/ClockUtils.h>

#include <iostream>

/**
 * The test uses 4 gpio pins:
 * - PB4: TIM3-CH1
 * - PA7: TIM3-CH2
 * - PC8: TIM3-CH3
 * - PB7: TIM4-CH2
 */

using namespace Boardcore;
using namespace miosix;

GpioPin pin1(GPIOB_BASE, 4);
GpioPin pin2(GPIOA_BASE, 7);
GpioPin pin3(GPIOC_BASE, 8);
GpioPin pin4(GPIOB_BASE, 7);

Servo s1(TIM3, TimerUtils::Channel::CHANNEL_1);
Servo s2(TIM3, TimerUtils::Channel::CHANNEL_2);
Servo s3(TIM3, TimerUtils::Channel::CHANNEL_3);
Servo s4(TIM4, TimerUtils::Channel::CHANNEL_2);

// Position to cycle through for the servo 1, 2 and 3
float positions[] = {0, 0.5, 1.0};
int lastPosition  = 0;

void moveServo()
{
    s1.setPosition(positions[lastPosition % 3]);
    s2.setPosition(positions[(lastPosition + 1) % 3]);
    s3.setPosition(positions[(lastPosition + 2) % 3]);

    lastPosition++;
}

int main()
{
    pin1.mode(Mode::ALTERNATE);
    pin1.alternateFunction(2);
    pin2.mode(Mode::ALTERNATE);
    pin2.alternateFunction(2);
    pin3.mode(Mode::ALTERNATE);
    pin3.alternateFunction(2);
    pin4.mode(Mode::ALTERNATE);
    pin4.alternateFunction(2);

    // Enable the timers
    s1.enable();
    s2.enable();
    s3.enable();
    s4.enable();

    // Start a periodic task to move the first three servos
    TaskScheduler scheduler;
    scheduler.addTask(&moveServo, 2 * 1000, 1);
    scheduler.start();

    // Control the fourth servo manually
    float percentage;

    printf("Please enter a percentage for the servo ");
    printf("or anything else to exit: ");

    while (std::cin >> percentage)
    {
        s4.setPosition(percentage / 100);

        printf("Position set to: %.2f%%\n", percentage);

        printf("Please enter a percentage for the servo ");
        printf("or anything else to exit: ");
    }
}
