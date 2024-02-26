/* Copyright (c) 2022-2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#pragma once

#include <logger/Logger.h>
#include <sensors/analog/Pitot/Pitot.h>
#include <utils/AeroUtils/AeroUtils.h>

#include "HILSensor.h"

template <int N_DATA>
struct PitotSimulatorData
{
    float deltaP[N_DATA];
    float airspeed[N_DATA];
};

/**
 * @brief fake pitot (differential pressure) sensor used for the simulation.
 *
 * This class is used to simulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a simulator.
 */
template <int N_DATA>
class HILPitot
    : public HILSensor<HILPitotData, PitotSimulatorData<N_DATA>, N_DATA>
{
    using Base = HILSensor<HILPitotData, PitotSimulatorData<N_DATA>, N_DATA>;

public:
    explicit HILPitot(const PitotSimulatorData<N_DATA> *sensorData)
        : HILSensor<HILPitotData, PitotSimulatorData<N_DATA>, N_DATA>(
              sensorData)
    {
    }

protected:
    HILPitotData updateData() override
    {
        HILPitotData tempData;
        {
            miosix::PauseKernelLock pkLock;
            tempData.deltaP   = Base::sensorData->deltaP[Base::sampleCounter];
            tempData.airspeed = Base::sensorData->airspeed[Base::sampleCounter];
            tempData.timestamp = Base::updateTimestamp();
        }
        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};
