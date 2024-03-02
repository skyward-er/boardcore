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
#include <algorithms/ADA/ADA_Algorithm0_ert_rtw/ADA_Algorithm0.h>
#include <algorithms/ReferenceValues.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <cmath>

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

ADA_Algorithm0 *ada;

void step()
{
    currentTimestamp += DELTA_T;
    auto currentPressure = reference.findNext(currentTimestamp);
    ADA_Algorithm0::ExtU_ADA_Algorithm0_T in{currentPressure};
    ada->setExternalInputs(&in);
    ada->step();
}

ReferenceValues getADAReferenceValues()
{
    return {DEFAULT_REFERENCE_ALTITUDE, DEFAULT_REFERENCE_PRESSURE,
            DEFAULT_REFERENCE_TEMPERATURE};
}

int main()
{
    ada = new ADA_Algorithm0();
    ada->initialize();

    printf("Starting, data duration: %f\n", reference.getDataDuration() / 1e6);

    while (currentTimestamp < reference.getDataDuration())
    {
        step();

        printf("%f, %f\n", currentTimestamp / 1e6,
               ada->getExternalOutputs().ADAState_d.verticalSpeed);
    }

    while (true)
        Thread::sleep(1000);
}
