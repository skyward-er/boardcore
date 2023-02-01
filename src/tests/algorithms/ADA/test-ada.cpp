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

#include <algorithms/ADA/ADA.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include "EuRoC-pressure-logs.h"
#include "ValueFollower.h"

using namespace miosix;
using namespace Boardcore;

ValueFollower reference(euRoCLogs, sizeof(euRoCLogs) / sizeof(PressureData));

static constexpr float DEFAULT_REFERENCE_ALTITUDE    = 160.0f;
static constexpr float DEFAULT_REFERENCE_PRESSURE    = 100022.4f;
static constexpr float KALMAN_INITIAL_ACCELERATION   = -500;
static constexpr float DEFAULT_REFERENCE_TEMPERATURE = 288.15f;

uint64_t currentTimestamp       = 0;
constexpr uint64_t DELTA_T      = 50 * 1e3;  // 50ms = 20Hz
constexpr float SAMPLING_PERIOD = 0.05;

ADA *ada;

void step()
{
    currentTimestamp += DELTA_T;
    auto currentPressure = reference.findNext(currentTimestamp);

    ada->update(currentPressure);
}

ReferenceValues getADAReferenceValues()
{
    return {DEFAULT_REFERENCE_ALTITUDE, DEFAULT_REFERENCE_PRESSURE,
            DEFAULT_REFERENCE_TEMPERATURE};
}

ADA::KalmanFilter::KalmanConfig getADAKalmanConfig()
{
    ADA::KalmanFilter::MatrixNN F_INIT;
    F_INIT << 1.0, SAMPLING_PERIOD, 0.5f * SAMPLING_PERIOD * SAMPLING_PERIOD,
        // cppcheck-suppress constStatement
        0.0, 1.0, SAMPLING_PERIOD, 0.0, 0.0, 1.0;
    ADA::KalmanFilter::MatrixPN H_INIT{1.0, 0.0, 0.0};
    ADA::KalmanFilter::MatrixNN P_INIT;
    // cppcheck-suppress constStatement
    P_INIT << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    ADA::KalmanFilter::MatrixNN Q_INIT;
    // cppcheck-suppress constStatement
    Q_INIT << 30.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 2.5f;
    ADA::KalmanFilter::MatrixPP R_INIT{4000.0f};
    ADA::KalmanFilter::MatrixNM G_INIT = ADA::KalmanFilter::MatrixNM::Zero();

    return {F_INIT,
            H_INIT,
            Q_INIT,
            R_INIT,
            P_INIT,
            G_INIT,
            ADA::KalmanFilter::CVectorN(DEFAULT_REFERENCE_PRESSURE, 0,
                                        KALMAN_INITIAL_ACCELERATION)};
}

int main()
{
    ada = new ADA(getADAKalmanConfig());
    ada->setReferenceValues(getADAReferenceValues());

    printf("Starting, data duration: %f\n", reference.getDataDuration() / 1e6);

    while (currentTimestamp < reference.getDataDuration())
    {
        step();

        printf("%f, %f\n", currentTimestamp / 1e6, ada->getState().aglAltitude);
    }

    while (true)
        Thread::sleep(1000);
}
