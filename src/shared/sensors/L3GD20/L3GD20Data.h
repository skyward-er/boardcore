
/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

struct L3GD20Data : public GyroscopeData
{
    L3GD20Data() : GyroscopeData{0, 0.0, 0.0, 0.0} {}

    L3GD20Data(uint64_t t, float x, float y, float z)
        : GyroscopeData{t, x, y, z}
    {
    }

    L3GD20Data(GyroscopeData gyr) : GyroscopeData(gyr) {}

    static std::string header()
    {
        return "gyro_timestamp,gyro_x,gyro_y,gyro_z\n";
    }

    void print(std::ostream& os) const
    {
        os << gyro_timestamp << "," << gyro_x << "," << gyro_y << "," << gyro_z
           << "\n";
    }
};