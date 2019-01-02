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

using namespace miosix;
using namespace HomeoneBoard;

int main()
{
    Cutter cutter;
    Thread::sleep(500);

    printf("Starting drogue cutter\n");
    cutter.startCutDrogue();

    Thread::sleep(500);
    printf("Stopping drogue cutter\n");
    cutter.stopCutDrogue();

    Thread::sleep(500);
    printf("Cutting main chute\n");
    cutter.startCutMainChute();

    Thread::sleep(500);
    printf("Starting drogue cutter\n");
    cutter.startCutDrogue();

    Thread::sleep(500);
    printf("Stop main chute\n");
    cutter.stopCutMainChute();

    Thread::sleep(500);
    printf("Stop drogue\n");
    cutter.stopCutDrogue();

    for (;;)
    {
        printf("END\n");
        Thread::sleep(10000);
    }
}