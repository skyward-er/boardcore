/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

// This prgram runs through a simulated flight and reports the apogee detection,
// while measuring the time elapsed

#define EIGEN_RUNTIME_NO_MALLOC

#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/TimerUtils.h>
#include <kalman/KalmanEigen.h>
#include <kernel/kernel.h>
#include <math/SkyQuaternion.h>
#include <util/util.h>

#include <iostream>

#include "test-kalman-data.h"

using namespace Boardcore;
using namespace miosix;
using namespace Eigen;

int main()
{
    // Setting pin mode for signaling ADA status
    {
        FastInterruptDisableLock dLock;

        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }

    printf("RUNNING...\n");

    // Timer for benchmarking purposes
    GeneralPurposeTimer<uint32_t> timer{TIM5};

    const int n = 3;
    const int p = 1;

    Matrix<float, n, n> F =
        (Matrix<float, n, n>(n, n) << 1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            .finished();
    // Output matrix
    Matrix<float, p, n> H{1, 0, 0};

    // Initial error covariance matrix
    Matrix<float, n, n> P = (Matrix<float, n, n>(n, n) << 0.1, 0.0, 0.0, 0.0,
                             0.1, 0.0, 0.0, 0.0, 0.1)
                                .finished();
    // Model variance matrix
    Matrix<float, n, n> Q = (Matrix<float, n, n>(n, n) << 0.01, 0.0, 0.0, 0.0,
                             0.01, 0.0, 0.0, 0.0, 0.01)
                                .finished();
    // Measurement variance
    Matrix<float, p, p> R{10};

    Matrix<float, n, 1> x0(INPUT[0], 0.0, 0.0);

    Matrix<float, p, 1> y(p);  // vector with p elements (only one in this case)

    KalmanEigen<float, n, p>::KalmanConfig config;
    config.F = F;
    config.H = H;
    config.Q = Q;
    config.R = R;
    config.P = P;
    config.x = x0;

    KalmanEigen<float, n, p> filter(config);

    float lastTime = 0.0;  // Variable to save the time of the last sample

    timer.reset();
    timer.enable();

    printf("%d %d \n", TIME.size(), INPUT.size());

    for (unsigned i = 1; i < TIME.size(); i++)
    {
        float time;  // Current time as read from csv file
        float T;     // Time elapsed between last sample and current one
        uint32_t tick1;
        uint32_t tick2;

        time = TIME[i];
        T    = time - lastTime;

        F(0, 1) = T;
        F(0, 2) = 0.5 * T * T;
        F(1, 2) = T;

        y(0) = INPUT[i];

        tick1 = timer.readCounter();

        filter.predict(F);

        if (!filter.correct(y))
        {
            printf("Correction failed at iteration : %u \n", i);
        }

        tick2 = timer.readCounter();

        printf("%u : %f \n", i,
               TimerUtils::toMilliSeconds(timer.getTimer(), tick2 - tick1));

        // printf("%f, %f, %f;\n", filter.getState()(0), filter.getState()(1),
        //       filter.getState()(2));

        // printf("%u \n", MemoryProfiling::getCurrentFreeStack());

        lastTime = time;

        if (filter.getState()(1) < 0)
        {
            printf("APOGEE DETECTED at iteration %u ! \n", i);
        }
    }

    // printf("Total time %d \n", timer.interval());

    return 0;
}
