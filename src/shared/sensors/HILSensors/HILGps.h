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

#include <cmath>

#include "HILSensor.h"

template <int N_DATA>
struct GPSSimulatorData
{
    float positionMeasures[N_DATA][3];
    float velocityMeasures[N_DATA][3];
    float fix;
    float num_satellites;
};

/**
 * @brief fake gps sensor used for the HILulation.
 *
 * This class is used to HILulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a HILulator.
 */
template <int N_DATA>
class HILGps : public HILSensor<HILGpsData, GPSSimulatorData<N_DATA>, N_DATA>
{
    using Base = HILSensor<HILGpsData, GPSSimulatorData<N_DATA>, N_DATA>;

public:
    explicit HILGps(const GPSSimulatorData<N_DATA> *sensorData)
        : HILSensor<HILGpsData, GPSSimulatorData<N_DATA>, N_DATA>(sensorData)
    {
    }

protected:
    HILGpsData updateData() override
    {
        HILGpsData tempData;
        {
            miosix::PauseKernelLock pkLock;

            tempData.latitude =
                Base::sensorData->positionMeasures[Base::sampleCounter][0];
            tempData.longitude =
                Base::sensorData->positionMeasures[Base::sampleCounter][1];
            tempData.height =
                Base::sensorData->positionMeasures[Base::sampleCounter][2];

            tempData.velocityNorth =
                Base::sensorData->velocityMeasures[Base::sampleCounter][0];
            tempData.velocityEast =
                Base::sensorData->velocityMeasures[Base::sampleCounter][1];
            tempData.velocityDown =
                Base::sensorData->velocityMeasures[Base::sampleCounter][2];
            tempData.speed =
                sqrtf(tempData.velocityNorth * tempData.velocityNorth +
                      tempData.velocityEast * tempData.velocityEast +
                      tempData.velocityDown * tempData.velocityDown);
            tempData.positionDOP = 0;

            tempData.fix = static_cast<uint8_t>(Base::sensorData->fix);
            tempData.satellites =
                static_cast<uint8_t>(Base::sensorData->num_satellites);

            tempData.gpsTimestamp = Base::updateTimestamp();
        }
        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};
