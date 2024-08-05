/* Copyright (c) 2020-2023 Skyward Experimental Rocketry
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

/**
 * @brief fake gps sensor used for the HILulation.
 *
 * This class is used to HILulate as near as possible the situation of the
 * OBSW during the flight, using fake sensors classes instead of the real
 * ones, taking their data from the data received from a HILulator.
 */
class HILGps : public HILSensor<HILGpsData>
{
public:
    HILGps(int n_data_sensor, void *sensorData)
        : HILSensor(n_data_sensor, sensorData)
    {
    }

protected:
    HILGpsData updateData() override
    {
        HILGpsData tempData;

        miosix::PauseKernelLock pkLock;
        HILConfig::SimulatorData::Gps *gps =
            reinterpret_cast<HILConfig::SimulatorData::Gps *>(sensorData);

        tempData.latitude =
            gps->positionMeasures[sampleCounter][0];  // divide by earth radius
        tempData.longitude = gps->positionMeasures[sampleCounter][1];
        tempData.height    = gps->positionMeasures[sampleCounter][2];

        tempData.velocityNorth = gps->velocityMeasures[sampleCounter][0];
        tempData.velocityEast  = gps->velocityMeasures[sampleCounter][1];
        tempData.velocityDown  = gps->velocityMeasures[sampleCounter][2];
        tempData.speed = sqrtf(tempData.velocityNorth * tempData.velocityNorth +
                               tempData.velocityDown * tempData.velocityDown);
        tempData.positionDOP = 0;

        tempData.fix        = static_cast<uint8_t>(gps->fix);
        tempData.satellites = static_cast<uint8_t>(gps->num_satellites);

        tempData.gpsTimestamp = updateTimestamp();

        Boardcore::Logger::getInstance().log(tempData);

        return tempData;
    }
};
