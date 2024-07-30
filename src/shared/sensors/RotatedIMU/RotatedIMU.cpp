/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "RotatedIMU.h"

using namespace Eigen;

namespace Boardcore
{

RotatedIMU::RotatedIMU(SampleIMUFunction sampleImuFunction)
    : sampleImu(std::move(sampleImuFunction))
{
}

void RotatedIMU::addAccTransformation(const Matrix3f& t) { accT = t * accT; }
void RotatedIMU::addGyroTransformation(const Matrix3f& t) { gyroT = t * gyroT; }
void RotatedIMU::addMagTransformation(const Matrix3f& t) { magT = t * magT; }

void RotatedIMU::resetTransformations()
{
    accT  = Matrix3f::Identity();
    gyroT = Matrix3f::Identity();
    magT  = Matrix3f::Identity();
}

Matrix3f RotatedIMU::rotateAroundX(float angle)
{
    Matrix3f rotation;
    angle = angle * EIGEN_PI / 180.f;

    // clang-format off
    rotation = Matrix3f{{1,     0,           0},
                        {0,     cosf(angle), -sinf(angle)},
                        {0,     sinf(angle), cosf(angle)}};
    // clang-format on

    return rotation;
}

Matrix3f RotatedIMU::rotateAroundY(float angle)
{
    Matrix3f rotation;
    angle = angle * EIGEN_PI / 180.f;

    // clang-format off
    rotation = Matrix3f{{cosf(angle),   0,  sinf(angle)},
                        {0,             1,  0},
                        {-sinf(angle),  0,  cosf(angle)}};
    // clang-format on

    return rotation;
}

Matrix3f RotatedIMU::rotateAroundZ(float angle)
{
    Matrix3f rotation;
    angle = angle * EIGEN_PI / 180.f;

    // clang-format off
    rotation = Matrix3f{{cosf(angle),   -sinf(angle),   0},
                        {sinf(angle),   cosf(angle),    0},
                        {0,             0,              1}};
    // clang-format on

    return rotation;
}

IMUData RotatedIMU::sampleImpl()
{
    auto imuData = sampleImu();

    AccelerometerData& accData = imuData;
    GyroscopeData& gyroData    = imuData;
    MagnetometerData& magData  = imuData;

    Vector3f rotatedAcc  = accT * static_cast<Vector3f>(accData);
    Vector3f rotatedGyro = gyroT * static_cast<Vector3f>(gyroData);
    Vector3f rotatedMag  = magT * static_cast<Vector3f>(magData);

    auto acc                  = AccelerometerData{rotatedAcc};
    acc.accelerationTimestamp = accData.accelerationTimestamp;

    auto gyro                  = GyroscopeData{rotatedGyro};
    gyro.angularSpeedTimestamp = gyroData.angularSpeedTimestamp;

    auto mag                   = MagnetometerData{rotatedMag};
    mag.magneticFieldTimestamp = magData.magneticFieldTimestamp;

    return {acc, gyro, mag};
}

}  // namespace Boardcore
