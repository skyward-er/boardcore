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

#include <actuators/stepper/Stepper.h>

using namespace miosix;
using namespace Boardcore;

GpioPin directionPin = GpioPin(GPIOC_BASE, 12);
GpioPin stepPin      = GpioPin(GPIOB_BASE, 4);
GpioPin enablePin    = GpioPin(GPIOC_BASE, 10);
GpioPin resetPin     = GpioPin(GPIOE_BASE, 5);
GpioPin ms1Pin       = GpioPin(GPIOE_BASE, 6);
GpioPin ms2Pin       = GpioPin(GPIOE_BASE, 2);
GpioPin ms3Pin       = GpioPin(GPIOE_BASE, 4);

void setMicroStepping(Stepper& stepper, int16_t microStep)
{
    switch (microStep)
    {
        case 1:
            ms1Pin.low();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case 2:
            ms1Pin.high();
            ms2Pin.low();
            ms3Pin.low();
            break;
        case 4:
            ms1Pin.low();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case 8:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.low();
            break;
        case 16:
            ms1Pin.high();
            ms2Pin.high();
            ms3Pin.high();
            break;
        default:
            printf("Microsteps value not available!\n");
            return;
    }
    stepper.setMicroStepping(microStep);
}

void doRoutine(Stepper& stepper)
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
    directionPin.mode(Mode::OUTPUT);
    stepPin.mode(Mode::OUTPUT);
    resetPin.mode(Mode::OUTPUT);
    resetPin.high();
    ms3Pin.mode(Mode::OUTPUT);
    ms2Pin.mode(Mode::OUTPUT);
    ms1Pin.mode(Mode::OUTPUT);
    enablePin.mode(Mode::OUTPUT);

    Stepper stepper(stepPin, directionPin, 1, 1.8, false, 8,
                    Stepper::PinConfiguration::COMMON_CATHODE, enablePin);

    printf("The stepper is now disabled, waiting 2 seconds...\n");
    delayMs(2 * 1000);

    while (true)
    {
        printf("Press something to start\n");
        scanf("%*s");

        printf("\t1x micro stepping\n");
        setMicroStepping(stepper, 8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t2x micro stepping\n");
        setMicroStepping(stepper, 8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t4x micro stepping\n");
        setMicroStepping(stepper, 8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t8x micro stepping\n");
        setMicroStepping(stepper, 8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");

        delayMs(1000);

        printf("\t16x micro stepping\n");
        setMicroStepping(stepper, 8);
        doRoutine(stepper);
        printf("The stepper is now disabled\n");
    }
}
