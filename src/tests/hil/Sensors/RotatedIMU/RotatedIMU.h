/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include "RotatedIMUData.h"

namespace HILTest
{
/**
 * @brief Creates a "fake" sensor which allows the user
 * to transform the values with transformation matrices.
 * The default configuration (thus the reset one)
 * comprehends identity matrices as transformation ones.
 *
 * @warning This is NOT a thread safe class
 */
class RotatedIMU : public Boardcore::Sensor<RotatedIMUData>
{
public:
    /**
     * @brief Construct a new Rotated IMU object
     *
     * @param accSampleFunction Function to call to retrieve an accelerometer
     * sample
     * @param magSampleFunction Function to call to retrieve a magnetometer
     * sample
     * @param gyroSampleFunction Function to call to retrieve a gyroscope sample
     */
    RotatedIMU(
        std::function<Boardcore::AccelerometerData()> accSampleFunction = []()
        { return Boardcore::AccelerometerData{}; },
        std::function<Boardcore::MagnetometerData()> magSampleFunction = []()
        { return Boardcore::MagnetometerData{}; },
        std::function<Boardcore::GyroscopeData()> gyroSampleFunction = []()
        { return Boardcore::GyroscopeData{}; });

    /**
     * @brief Multiplies the current accelerometer transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addAccTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Multiplies the current magnetometer transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addMagTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Multiplies the current gyroscope transformation
     * @param transformation Transformation matrix to be multiplied to the
     * current one
     */
    void addGyroTransformation(const Eigen::Matrix3f& t);

    /**
     * @brief Resets all the transformations to the original (Identity) ones
     */
    void resetTransformations();

    bool init() override { return true; }
    bool selfTest() override { return true; }

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
    RotatedIMUData sampleImpl() override;

    // Functions to sample the under neath sensors
    std::function<Boardcore::AccelerometerData()> accSample;
    std::function<Boardcore::MagnetometerData()> magSample;
    std::function<Boardcore::GyroscopeData()> gyroSample;

    // Transformation matrices
    Eigen::Matrix3f accT;
    Eigen::Matrix3f magT;
    Eigen::Matrix3f gyroT;
};
}  // namespace HILTest