/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

struct LIS3DSHData : public AccelerometerData, public TemperatureData
{
    using MeterPerSecondSquared = Units::Acceleration::MeterPerSecondSquared;

    LIS3DSHData()
        : AccelerometerData{0, MeterPerSecondSquared(0),
                            MeterPerSecondSquared(0), MeterPerSecondSquared(0)},
          TemperatureData{0, 0.0}
    {
    }

    LIS3DSHData(uint64_t t, MeterPerSecondSquared x, MeterPerSecondSquared y,
                MeterPerSecondSquared z, float temp)
        : AccelerometerData{t, x, y, z}, TemperatureData{t, temp}
    {
    }

    LIS3DSHData(AccelerometerData acc, TemperatureData temp)
        : AccelerometerData{acc.accelerationTimestamp, acc.accelerationX,
                            acc.accelerationY, acc.accelerationZ},
          TemperatureData{temp.temperatureTimestamp, temp.temperature}
    {
    }

    static std::string header()
    {
        return "accelerationTimestamp,accelerationX,accelerationY,"
               "accelerationZ,temp_"
               "timestamp,"
               "temp\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << temperatureTimestamp << "," << temperature << "\n";
    }
};

}  // namespace Boardcore
