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
    lhs.accelerationX = rhs[0];
    lhs.accelerationY = rhs[1];
    lhs.accelerationZ = rhs[2];
}

void operator<<(GyroscopeData& lhs, const Vector3f& rhs)
{
    lhs.angularVelocityX = rhs[0];
    lhs.angularVelocityY = rhs[1];
    lhs.angularVelocityZ = rhs[2];
}

void operator<<(MagnetometerData& lhs, const Vector3f& rhs)
{
    lhs.magneticFieldX = rhs[0];
    lhs.magneticFieldY = rhs[1];
    lhs.magneticFieldZ = rhs[2];
}

void operator>>(const AccelerometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.accelerationX;
    rhs[1] = lhs.accelerationY;
    rhs[2] = lhs.accelerationZ;
}

void operator>>(const GyroscopeData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.angularVelocityX;
    rhs[1] = lhs.angularVelocityY;
    rhs[2] = lhs.angularVelocityZ;
}

void operator>>(const MagnetometerData& lhs, Vector3f& rhs)
{
    rhs[0] = lhs.magneticFieldX;
    rhs[1] = lhs.magneticFieldY;
    rhs[2] = lhs.magneticFieldZ;
}

}  // namespace Boardcore
