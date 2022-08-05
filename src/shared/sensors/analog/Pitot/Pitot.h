/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio, Arturo Benedetti
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

#include <sensors/Sensor.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <functional>

#include "PitotData.h"

namespace Boardcore
{

class Pitot : public Sensor<PitotData>
{
public:
    Pitot(std::function<PressureData()> getPitotPressure,
          std::function<float()> getStaticPressure)
        : getPitotPressure(getPitotPressure),
          getStaticPressure(getStaticPressure)
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

    PitotData sampleImpl() override
    {
        float airDensity =
            Aeroutils::relDensity(getStaticPressure(), REFERENCE_PRESSURE,
                                  REFERENCE_ALTITUDE, REFERENCE_TEMPERATURE);
        if (airDensity != 0.0)
        {
            PitotData pitotSpeed;

            pitotSpeed.timestamp = getPitotPressure().pressureTimestamp;
            pitotSpeed.airspeed =
                sqrtf(2 * fabs(getPitotPressure().pressure) / airDensity);

            return pitotSpeed;
        }

        return lastSample;
    }

private:
    std::function<PressureData()> getPitotPressure;
    std::function<float()> getStaticPressure;

    static constexpr float REFERENCE_PRESSURE = 100000.0;  // Milan air pressure
    static constexpr float REFERENCE_ALTITUDE = 130.0;     // Milan altitude
    static constexpr float REFERENCE_TEMPERATURE = 273.15 + 25.0;  // 25°
};

}  // namespace Boardcore
