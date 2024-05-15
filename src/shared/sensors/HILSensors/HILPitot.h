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

#include <algorithms/ReferenceValues.h>
#include <logger/Logger.h>
#include <sensors/analog/Pitot/Pitot.h>
#include <utils/AeroUtils/AeroUtils.h>

#include "HILSensor.h"

namespace Boardcore
{

/**
 * @brief fake pitot (differential pressure) sensor used for the simulation.
 *
 * This class is used to simulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a simulator.
 */
template <int N_DATA>
class HILPitot : public Mock::HILSensor<PitotData, PitotSimulatorData<N_DATA>>
{
    using Base = Mock::HILSensor<PitotData, PitotSimulatorData<N_DATA>>;

public:
    explicit HILPitot(const PitotSimulatorData<N_DATA> *sensorData)
        : Base(sensorData),
          pitot(
              [&]()
              {
                  // Assemble total pressure as sum of staticPressure and deltaP
                  return Base::sensorData->staticPressure[Base::sampleCounter] +
                         Base::sensorData->deltaP[Base::sampleCounter];
              },
              [&]()
              { return Base::sensorData->staticPressure[Base::sampleCounter]; })
    {
    }

    void setReferenceValues(const Boardcore::ReferenceValues reference)
    {
        pitot.setReferenceValues(reference);
    }

    Boardcore::ReferenceValues getReferenceValues()
    {
        return pitot.getReferenceValues();
    }

protected:
    PitotData updateData() override
    {
        PitotData tempData;
        pitot.sample();
        {
            miosix::PauseKernelLock pkLock;
            auto sample        = pitot.getLastSample();
            tempData.deltaP    = sample.deltaP;
            tempData.airspeed  = sample.airspeed;
            tempData.timestamp = Base::updateTimestamp();
        }
        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }

    Boardcore::Pitot pitot;
};
}  // namespace Boardcore