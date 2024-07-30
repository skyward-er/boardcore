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

#pragma once

#include <sensors/Sensor.h>

#include <Eigen/Eigen>
#include <functional>

#include "IMUData.h"

namespace Boardcore
{

/**
 * @brief A software IMU sensor that allows applying transformations to the data
 * after sampling via a callback. Defaults to identity transformations.
 */
class RotatedIMU : public Boardcore::Sensor<IMUData>
{
public:
    using SampleIMUFunction = std::function<IMUData()>;

    /**
     * @brief Construct a new Rotated IMU object
     *
     * @param sampleFunction Callback to retrieve accelerometer, magnetometer
     * and gyroscope data
     */
    explicit RotatedIMU(SampleIMUFunction sampleFunction);

    bool init() override { return true; }
    bool selfTest() override { return true; }

    /**
     * @brief Multiplies the current accelerometer transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addAccTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Multiplies the current gyroscope transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addGyroTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Multiplies the current magnetometer transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addMagTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Resets all the transformations to the original (Identity) ones
     */
    void resetTransformations();

    /**
     * @brief Creates a rotation matrix around the X axis
     * @param angle Angle in degrees
     */
    static Eigen::Matrix3f rotateAroundX(float angle);

    /**
     * @brief Creates a rotation matrix around the Y axis
     * @param angle Angle in degrees
     */
    static Eigen::Matrix3f rotateAroundY(float angle);

    /**
     * @brief Creates a rotation matrix around the Z axis
     * @param angle Angle in degrees
     */
    static Eigen::Matrix3f rotateAroundZ(float angle);

protected:
    IMUData sampleImpl() override;

private:
    SampleIMUFunction sampleImu;

    // Transformation matrices
    Eigen::Matrix3f accT  = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f gyroT = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f magT  = Eigen::Matrix3f::Identity();
};

}  // namespace Boardcore
