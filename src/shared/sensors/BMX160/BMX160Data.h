/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Davide Mor
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "sensors/SensorData.h"

/// @brief BMX160 Data struct.
struct BMX160Data : public AccelerometerData, GyroscopeData, MagnetometerData
{
    /// @brief Default constructor.
    BMX160Data()
        : AccelerometerData{0, 0.0, 0.0, 0.0}, GyroscopeData{0, 0.0, 0.0, 0.0},
          MagnetometerData{0, 0.0, 0.0, 0.0}
    {
    }

    /// @brief Comodity constructor.
    BMX160Data(AccelerometerData acc, GyroscopeData gyr, MagnetometerData mag)
        : AccelerometerData(acc), GyroscopeData(gyr), MagnetometerData(mag)
    {
    }

    static std::string header()
    {
        return "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,"
               "gyro_y,"
               "gyro_z,mag_timestamp,mag_x,mag_y,mag_z\n";
    }

    void print(std::ostream& os) const
    {
        os << accel_timestamp << "," << accel_x << "," << accel_y << ","
           << accel_z << "," << gyro_timestamp << "," << gyro_x << "," << gyro_y
           << "," << gyro_z << "," << mag_timestamp << "," << mag_x << ","
           << mag_y << "," << mag_z << "\n";
    }
};

struct BMX160Temerature : public TemperatureData
{
    static std::string header() { return "temp_timestamp,temperature\n"; }

    void print(std::ostream& os) const
    {
        os << temp_timestamp << "," << temp << "\n";
    }
};