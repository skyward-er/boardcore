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
    using namespace Units::Acceleration;

    lhs.accelerationX = MeterPerSecondSquared(rhs[0]);
    lhs.accelerationY = MeterPerSecondSquared(rhs[1]);
    lhs.accelerationZ = MeterPerSecondSquared(rhs[2]);
}

void operator<<(Eigen::Vector3f& lhs, const AccelerometerData& rhs)
{
    lhs[0] = rhs.accelerationX.value();
    lhs[1] = rhs.accelerationY.value();
    lhs[2] = rhs.accelerationZ.value();
}

void operator<<(GyroscopeData& lhs, const Vector3f& rhs)
{
    using namespace Units::Angle;

    lhs.angularSpeedX = Degree(rhs[0]);
    lhs.angularSpeedY = Degree(rhs[1]);
    lhs.angularSpeedZ = Degree(rhs[2]);
}

void operator<<(Eigen::Vector3f& lhs, const GyroscopeData& rhs)
{
    lhs[0] = rhs.angularSpeedX.value();
    lhs[1] = rhs.angularSpeedY.value();
    lhs[2] = rhs.angularSpeedZ.value();
}

void operator<<(MagnetometerData& lhs, const Vector3f& rhs)
{
    lhs.magneticFieldX = rhs[0];
    lhs.magneticFieldY = rhs[1];
    lhs.magneticFieldZ = rhs[2];
}

void operator<<(Eigen::Vector3f& lhs, const MagnetometerData& rhs)
{
    lhs[0] = rhs.magneticFieldX;
    lhs[1] = rhs.magneticFieldY;
    lhs[2] = rhs.magneticFieldZ;
}

void operator>>(const AccelerometerData& lhs, Eigen::Vector3f& rhs)
{
    rhs << lhs;
}

void operator>>(const Eigen::Vector3f& lhs, AccelerometerData& rhs)
{
    rhs << lhs;
}

void operator>>(const GyroscopeData& lhs, Eigen::Vector3f& rhs) { rhs << lhs; }

void operator>>(const Eigen::Vector3f& lhs, GyroscopeData& rhs) { rhs << lhs; }

void operator>>(const MagnetometerData& lhs, Eigen::Vector3f& rhs)
{
    rhs << lhs;
}

void operator>>(const Eigen::Vector3f& lhs, MagnetometerData& rhs)
{
    rhs << lhs;
}

}  // namespace Boardcore
