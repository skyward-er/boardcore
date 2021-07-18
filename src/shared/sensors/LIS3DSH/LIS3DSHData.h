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

#include "sensors/SensorData.h"

struct LIS3DSHData : public AccelerometerData, public TemperatureData
{
    LIS3DSHData() : AccelerometerData{0, 0.0, 0.0, 0.0}, TemperatureData{0, 0.0}
    {
    }

    LIS3DSHData(uint64_t t, float x, float y, float z, float temp)
        : AccelerometerData{t, x, y, z},
          TemperatureData{t, temp}
    {
    }

    LIS3DSHData(AccelerometerData acc, TemperatureData temp)
        : AccelerometerData{acc.accel_timestamp, acc.accel_x, acc.accel_y, acc.accel_z},
          TemperatureData{temp.temp_timestamp, temp.temp}
    {
    }

    static std::string header()
    {
        return "accel_timestamp,accel_x,accel_y,accel_z,temp_timestamp,temp\n";
    }

    void print(std::ostream& os) const
    {
        os << accel_timestamp << "," << accel_x << "," << accel_y << "," << accel_z
           << "," << temp_timestamp << "," << temp << "\n";
    }
};