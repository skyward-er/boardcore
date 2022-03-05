/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/stepper/Stepper.h>

using namespace miosix;
using namespace Boardcore;

miosix::GpioPin directionPin = miosix::GpioPin(GPIOC_BASE, 13);
miosix::GpioPin stepPin      = miosix::GpioPin(GPIOC_BASE, 14);
miosix::GpioPin resetPin     = miosix::GpioPin(GPIOE_BASE, 5);
miosix::GpioPin ms1Pin       = miosix::GpioPin(GPIOE_BASE, 6);
miosix::GpioPin ms2Pin       = miosix::GpioPin(GPIOE_BASE, 2);
miosix::GpioPin ms3Pin       = miosix::GpioPin(GPIOE_BASE, 4);
miosix::GpioPin enablePin    = miosix::GpioPin(GPIOC_BASE, 15);

void doRoutine(Stepper stepper)
{
    stepper.enable();

    for (int i = 1; i < 9; i++)
    {
        stepper.setSpeed(i);
        printf("Current speed: %d\n", i);

        stepper.setPositionDeg(360);
        printf("This are 360°\n");

        delayMs(500);

        stepper.setPositionDeg(180);
        printf("This are 180°\n");

        delayMs(500);

        stepper.moveDeg(-180);
        printf("This are 0°\n");

        delayMs(500);

        printf("1 full rotation\n");
        stepper.setPositionDeg(-360);
        delayMs(500);
        stepper.setPosition(0);

        printf("Current position: %f\n", stepper.getCurrentDegPosition());
        delayMs(1000);
    }

    stepper.enable();
}

int main()
{
    directionPin.mode(miosix::Mode::OUTPUT);
    stepPin.mode(miosix::Mode::OUTPUT);
    resetPin.mode(miosix::Mode::OUTPUT);
    resetPin.high();
    ms3Pin.mode(miosix::Mode::OUTPUT);
    ms2Pin.mode(miosix::Mode::OUTPUT);
    ms1Pin.mode(miosix::Mode::OUTPUT);
    enablePin.mode(miosix::Mode::OUTPUT);

    Stepper stepper(stepPin, directionPin, 1, 1.8, false,
                    Stepper::Microstep::MICROSTEP_1, enablePin, ms1Pin, ms2Pin,
                    ms3Pin);

    printf("The stepper is now disabled, waiting 2 seconds...\n");
    delayMs(2 * 1000);

    while (true)
    {
        printf("Press something to start\n");
        getchar();

        printf("\t1x microstepping\n");
        stepper.setMicrostepping(Stepper::Microstep::MICROSTEP_1);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t2x microstepping\n");
        stepper.setMicrostepping(Stepper::Microstep::MICROSTEP_2);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t4x microstepping\n");
        stepper.setMicrostepping(Stepper::Microstep::MICROSTEP_4);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t8x microstepping\n");
        stepper.setMicrostepping(Stepper::Microstep::MICROSTEP_8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t16x microstepping\n");
        stepper.setMicrostepping(Stepper::Microstep::MICROSTEP_16);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");
    }
}