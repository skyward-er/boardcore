/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Riccardo Musso
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

struct LIS3MDLData : public MagnetometerData, public TemperatureData
{
    LIS3MDLData() : MagnetometerData{0, 0.0, 0.0, 0.0}, TemperatureData{0, 0.0}
    {
    }

    LIS3MDLData(uint64_t t, float mx, float my, float mz, float deg)
        : MagnetometerData{t, mx, my, mz}, TemperatureData{t, deg}

    {
    }

    LIS3MDLData(MagnetometerData magData, TemperatureData tempData)
        : MagnetometerData(magData), TemperatureData(tempData)

    {
    }

    static std::string header()
    {
        return "magneticFieldTimestamp,magneticFieldX,magneticFieldY,"
               "magneticFieldZ,"
               "temperatureTimestamp,"
               "temp\n";
    }

    void print(std::ostream& os) const
    {
        os << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << ","
           << temperatureTimestamp << "," << temperature << "\n";
    }
};

}  // namespace Boardcore
