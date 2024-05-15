/* Copyright (c) 2020-2024 Skyward Experimental Rocketry
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

#include "HILSensor.h"

namespace Boardcore
{
/**
 * @brief fake gyroscope sensor used for the simulation.
 *
 * This class is used to simulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a simulator.
 */
template <int N_DATA>
class HILGyroscope
    : public Mock::HILSensor<GyroscopeData, GyroscopeSimulatorData<N_DATA>>
{
    using Base = Mock::HILSensor<GyroscopeData, GyroscopeSimulatorData<N_DATA>>;

public:
    explicit HILGyroscope(const GyroscopeSimulatorData<N_DATA> *sensorData)
        : Base(sensorData)
    {
    }

protected:
    GyroscopeData updateData() override
    {
        GyroscopeData tempData;
        {
            miosix::PauseKernelLock pkLock;
            tempData.angularSpeedX =
                Base::sensorData->measures[Base::sampleCounter][0];
            tempData.angularSpeedY =
                Base::sensorData->measures[Base::sampleCounter][1];
            tempData.angularSpeedZ =
                Base::sensorData->measures[Base::sampleCounter][2];
            tempData.angularSpeedTimestamp = Base::updateTimestamp();
        }
        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};
}  // namespace Boardcore