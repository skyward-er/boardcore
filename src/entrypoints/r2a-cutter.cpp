/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include "boards/Homeone/DeploymentController/ThermalCutter/Cutter.h"

using namespace HomeoneBoard;

typedef miosix::Gpio<GPIOG_BASE, 11> btn;

static const int BUTTON_SLEEP = 10;

using miosix::Thread;

void awaitButton(int time)
{
    int pressed_for = 0;
    do
    {
        if (btn::value() == 0)
        {   
            pressed_for += BUTTON_SLEEP;        
        }
        else
        {
            pressed_for = 0;
        }
        miosix::Thread::sleep(BUTTON_SLEEP);
    } while (pressed_for < time);
}

int main()
{
    miosix::ledOff();
    btn::mode(miosix::Mode::INPUT);
    Cutter cutter;

    printf("Press the button for 3 seconds to enable the cutter\n");

    awaitButton(3000);

    printf("Cutter enabled\n");
    miosix::ledOn();
    cutter.startCutDrogue();  // TIM4-CH1

    Thread::sleep(5000);

    awaitButton(50);
    printf("Cutter disabled\n");
    cutter.stopCutDrogue();
    miosix::ledOff();
    for(;;)
    {
        printf("End\n");
        Thread::sleep(10000);
    }
}