/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

int main()
{
    GpioPin stepPin(GPIOB_BASE, 13);
    GpioPin directionPin(GPIOB_BASE, 14);

    stepPin.mode(Mode::OUTPUT);
    directionPin.mode(Mode::OUTPUT);

    Stepper stepper(stepPin, directionPin, 1, 1.8 / 16, false);

    while (true)
    {
        printf("Press something to start\n");
        getchar();

        stepper.setPositionDeg(360);
        printf("This are 360°\n");
        delayMs(2 * 1000);

        stepper.setPositionDeg(180);
        printf("This are 180°\n");
        delayMs(2 * 1000);

        stepper.moveDeg(-180);
        printf("This are 0°\n");
        delayMs(2 * 1000);

        stepper.setSpeed(4);
        printf("Set speed to 4rev/sec\n");

        stepper.setPositionDeg(-360);
        printf("This are -360°\n");
        delayMs(2 * 1000);

        stepper.setPositionDeg(-180);
        printf("This are -180°\n");
        delayMs(2 * 1000);

        stepper.setPositionDeg(0);
        printf("This are 0°\n");
        delayMs(2 * 1000);

        stepper.setPositionDeg(360 * 10);
        printf("This are 3600°\n");
        delayMs(2 * 1000);

        stepper.setPositionDeg(0);
        printf("This are 0°\n");

        printf("Now 6 steps of 60°\n");
        for (int i = 0; i < 6; i++)
        {
            stepper.moveDeg(60);
            delayMs(500);
        }

        printf("Now 6 steps of -60°\n");
        for (int i = 0; i < 6; i++)
        {
            stepper.moveDeg(-60);
            delayMs(500);
        }

        printf("\n");
    }
}