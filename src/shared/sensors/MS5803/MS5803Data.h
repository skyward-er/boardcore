/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <sensors/SensorData.h>

namespace Boardcore
{

/**
 * @brief MS5803 calibration data. See page 13 of datasheet for more details.
 */
struct MS5803CalibrationData
{
    uint16_t sens     = 0;
    uint16_t off      = 0;
    uint16_t tcs      = 0;
    uint16_t tco      = 0;
    uint16_t tref     = 0;
    uint16_t tempsens = 0;
};

struct MS5803Data : public PressureData, TemperatureData
{

    MS5803Data() : PressureData{0, Pascal(0)}, TemperatureData{0, 0.0} {}

    MS5803Data(uint64_t pressureTimestamp, Pascal pressure,
               uint64_t temperatureTimestamp, float temperature)
        : PressureData{pressureTimestamp, pressure},
          TemperatureData{temperatureTimestamp, temperature}
    {
    }

    static std::string header()
    {
        return "pressureTimestamp,pressure,temperatureTimestamp,temperature\n";
    }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << temperatureTimestamp << "," << temperature << "\n";
    }
};

}  // namespace Boardcore
