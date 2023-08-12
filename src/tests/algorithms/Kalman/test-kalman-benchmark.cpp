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

#include <algorithms/Kalman/Kalman.h>
#include <drivers/timer/TimestampTimer.h>
#include <kernel/kernel.h>
#include <util/util.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

#include "test-kalman-data.h"

using namespace Boardcore;
using namespace miosix;
using namespace Eigen;

int main()
{
    printf("RUNNING...\n");

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

    Matrix<float, n, 1> G = Matrix<float, n, 1>::Zero();

    Matrix<float, n, 1> x0(INPUT[0], 0.0, 0.0);

    Matrix<float, p, 1> y(p);  // vector with p elements (only one in this case)

    Kalman<float, n, p>::KalmanConfig config;
    config.F = F;
    config.H = H;
    config.Q = Q;
    config.R = R;
    config.P = P;
    config.G = G;
    config.x = x0;

    Kalman<float, n, p> filter(config);

    float lastTime = 0.0;  // Variable to save the time of the last sample

    uint64_t startTime  = TimestampTimer::getTimestamp();
    bool apogeeDetected = false;

    printf("%d %d \n", TIME.size(), INPUT.size());

    for (unsigned i = 1; i < TIME.size(); i++)
    {
        float time;  // Current time as read from csv file
        float T;     // Time elapsed between last sample and current one

        time = TIME[i];
        T    = time - lastTime;

        F(0, 1) = T;
        F(0, 2) = 0.5 * T * T;
        F(1, 2) = T;

        y(0) = INPUT[i];

        filter.predictNewF(F);

        if (!filter.correct(y))
            printf("Correction failed at iteration : %u \n", i);

        lastTime = time;

        if (filter.getState()(1) < 0 && !apogeeDetected)
        {
            printf("APOGEE DETECTED at iteration %u ! \n", i);
            apogeeDetected = true;
        }
    }

    uint64_t endTime = TimestampTimer::getTimestamp();
    printf("Total time %f \n", (endTime - startTime) / 1e6);

    while (true)
        Thread::sleep(1000);
}
