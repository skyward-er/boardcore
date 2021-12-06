/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include "SensorDataExtra.h"

using namespace Eigen;

namespace Boardcore
{

void operator<<(AccelerometerData& lhs, const Vector3f& rhs)
{
    lhs.accel_x = rhs[0];
    lhs.accel_y = rhs[1];
    lhs.accel_z = rhs[2];
}

void operator<<(GyroscopeData& lhs, const Vector3f& rhs)
{
    lhs.gyro_x = rhs[0];
    lhs.gyro_y = rhs[1];
    lhs.gyro_z = rhs[2];
}

void operator<<(MagnetometerData& lhs, const Vector3f& rhs)
{
    lhs.mag_x = rhs[0];
    lhs.mag_y = rhs[1];
    lhs.mag_z = rhs[2];
}

void operator>>(const AccelerometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.accel_x;
    rhs[1] = lhs.accel_y;
    rhs[2] = lhs.accel_z;
}

void operator>>(const GyroscopeData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.gyro_x;
    rhs[1] = lhs.gyro_y;
    rhs[2] = lhs.gyro_z;
}

void operator>>(const MagnetometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.mag_x;
    rhs[1] = lhs.mag_y;
    rhs[2] = lhs.mag_z;
}

}  // namespace Boardcore
