/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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

// This prgram runs through a simulated flight and reports the apogee detection,
// while measuring the time elapsed


// RESULT: Update operation 0.0319 on average

#pragma once
#include <Common.h>
#include <drivers/HardwareTimer.h>
#include <kalman/Kalman.h>
#include <iostream>
#include "util/util.h"
#include "test-kalman-data.h"

using namespace miosix;

typedef miosix::Gpio<GPIOG_BASE, 13> greenLed;
typedef miosix::Gpio<GPIOG_BASE, 14> redLed;

int main(int argc, char const* argv[])
{

    // Compiler, please shut up
    (void)argv;
    (void)argc;

    // Setting pin mode for signaling ADA status
    {
        FastInterruptDisableLock dLock;
        greenLed::mode(Mode::OUTPUT);
        redLed::mode(Mode::OUTPUT);
    }

    // Timer for benchmarking purposes
    HardwareTimer<uint32_t, 2>& timer = HardwareTimer<uint32_t, 2>::instance();

    // Instanciate matrices
    MatrixBase<float,3,3> P{0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
    MatrixBase<float,1,1> V2{10};
    MatrixBase<float,3,3> V1{0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    MatrixBase<float,1,3> C{1, 0, 0};
    MatrixBase<float,3,3> A{1, 0, 0, 0, 1, 0, 0, 0, 1};

    // Instanciate filter object
    Kalman<3,1> filter = Kalman<3,1>(A, C, V1, V2, P);

    float last_time = 0.0;  // Variable to save the time of the last sample
    float time;             // Current time as read from csv file
    float T;                // Time elapsed between last sample and current one

    // miosix::Timer timer;
    // timer.start();
    timer.start();
    uint32_t tick1;
    uint32_t tick2;

    for (unsigned i = 0; i < TIME.size(); i++)
    {
        if (i == 0)
        {
            filter.X(0,0) = INPUT[0];
            continue;
        }
        time = TIME[i];
        T    = time - last_time;

        filter.A(0, 1) = T;
        filter.A(0, 2) = 0.5 * T * T;
        filter.A(1, 2) = T;

        MatrixBase<float,1,1> y{};
        y(0,0) = INPUT[i];

        tick1 = timer.tick();
        filter.update(y);
        tick2 = timer.tick();
        printf("%f \n", timer.toMilliSeconds(tick2 - tick1));
        // printf("%f, %f, %f;\n", filter.X(0,0), filter.X(1,0), filter.X(2,0));
        // std::cout << MemoryProfiling::getCurrentFreeStack() << "\n";
        last_time = time;
        if (filter.X(1,0) < 0)
        {
            greenLed::high();
            redLed::low();
        }
        else
        {
            greenLed::low();
            redLed::high();
        }
    }
    timer.stop();
    // printf("Total time %d \n", timer.interval());
    return 0;
}
