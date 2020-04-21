/* LSM9DS1 accelerometer + giroscope Driver
 *
 * Copyright (c) 2016,2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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

#include <miosix.h>

#include <fstream>

#include "../Sensor.h"

using std::ofstream;

// data Structs

// ACCELEROMETER + GYROSCOPE
struct lsm9ds1XLGSample
{
    uint64_t timestamp;
    Vec3 axelData;
    Vec3 gyroData;

    static std::string header()
    {
        return "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << axelData.getX() << "," << axelData.getY()
           << "," << axelData.getZ() << "," << gyroData.getX() << ","
           << gyroData.getY() << "," << gyroData.getZ() << "\n";
    }
};

// TEMPERATURE
struct lsm9ds1TSample
{
    uint64_t timestamp;
    float tempData;

    static std::string header() { return "timestamp,temp\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << tempData << "\n";
    }
};

// MAGNETOMETER
struct lsm9ds1MSample
{
    uint64_t timestamp;
    Vec3 magData;

    static std::string header() { return "timestamp,mag_x,mag_y,mag_z\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << magData.getX() << "," << magData.getY() << ","
           << magData.getZ() << "\n";
    }
};
