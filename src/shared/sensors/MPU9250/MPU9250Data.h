/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#ifndef SRC_SHARED_SENSORS_MPU9250_MPU9250DATA_H
#define SRC_SHARED_SENSORS_MPU9250_MPU9250DATA_H

#include <ostream>
#include "math/Vec3.h"

struct MPU9250Data
{
    long long timestamp;
    Vec3 accel;
    Vec3 gyro;
    Vec3 compass;
    float temp;

    static std::string header()
    {
        return "timestamp,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,compass_x,"
               "compass_y,compass_z\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << accel.getX() << "," << accel.getY() << ","
           << accel.getZ() << "," << gyro.getX() << "," << gyro.getY() << ","
           << gyro.getZ() << "," << compass.getX() << "," << compass.getY()
           << "," << compass.getZ() << "\n";
    }
};

#endif /* SRC_SHARED_SENSORS_MPU9250_MPU9250DATA_H */
