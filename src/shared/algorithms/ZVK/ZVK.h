/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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

#include <sensors/SensorData.h>
#include <utils/Constants.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <memory>

#include "ZVKConfig.h"
#include "ZVKState.h"

namespace Boardcore
{

class ZVK
{
public:
    // Index of velocity elements in the state.
    static constexpr uint8_t IDX_VEL = 0;

    // Index of acceleration elements in the state.
    static constexpr uint8_t IDX_ACC = 3;

    // Index of accelerometer0 bias elements in the state.
    static constexpr uint8_t IDX_BIAS_ACC_0 = 6;

    // Index of accelerometer1 bias elements in the state.
    static constexpr uint8_t IDX_BIAS_ACC_1 = 9;

    // Index of eulero angles elements in the state.
    static constexpr uint8_t IDX_EUL_ANG = 12;

    // Index of angular velocity elements in the state.
    static constexpr uint8_t IDX_VEL_ANG = 15;

    // Index of gyroscope0 bias elements in the state.
    static constexpr uint8_t IDX_BIAS_GYRO_0 = 18;

    // Index of gyroscope1 bias elements in the state.
    static constexpr uint8_t IDX_BIAS_GYRO_1 = 21;

    explicit ZVK(const ZVKConfig& config);

    /**
     * @brief Prediction.

     */
    void predict();

    /**
     * @brief Correction using zero rocket velocity.
     *
     */
    void correctZeroVel();

    /**
     * @brief Correction with accelerometer 0.
     *
     * @param acceleration Accelerometer data [m/s^2].
     */
    void correctAcc0(const Eigen::Vector3f& acceleration);

    /**
     * @brief Correction with accelerometer 1.
     *
     * @param acceleration Accelerometer data [m/s^2].
     */
    void correctAcc1(const Eigen::Vector3f& acceleration);

    /**
     * @brief Correction with gyroscope 0.
     *
     * @param angularVel gyroscope data data [rad/s].
     */
    void correctGyro0(const Eigen::Vector3f& angularVel);

    /**
     * @brief Correction with gyroscope 1.
     *
     * @param angularVel gyroscope data data [rad/s].
     */
    void correctGyro1(const Eigen::Vector3f& angularVel);

    /**
     * @brief Correction with accelerometer data.
     *
     * @param acceleration Vector with accelerometer data.
     */
    void correctAcc0(const AccelerometerData& acceleration);

    /**
     * @brief Correction with accelerometer data.
     *
     * @param acceleration Vector with accelerometer data.
     */
    void correctAcc1(const AccelerometerData& acceleration);

    /**
     * @brief Correction with gyroscope0 data.
     *
     * @param angularVel Vector with gyroscope data.
     */
    void correctGyro0(const GyroscopeData& angularVel);

    /**
     * @brief Correction with gyroscope0 data.
     *
     * @param angularVel Vector with gyroscope data.
     */
    void correctGyro1(const GyroscopeData& angularVel);

    /**
     * @return KF state.
     */
    ZVKState getState() const;

    /**
     * @param state KF state.
     */
    void setState(const ZVKState& state);

    /**
     * @return State vector [n e d ax ay az bax0 bay0 baz0 bax1 bay1 baz1 eax
     * eay eaz avx avy avz bgx0 bgy0 bgz0 bgx1 bgy1 bgz1]
     */
    Eigen::Matrix<float, 24, 1> getX() const;

    /**
     * @param state State vector [n e d ax ay az bax0 bay0 baz0 bax1 bay1 baz1
     * eax eay eaz avx avy avz bgx0 bgy0 bgz0 bgx1 bgy1 bgz1]
     */
    void setX(const Eigen::Matrix<float, 24, 1>& x);

private:
    ZVKConfig config;

    // State
    Eigen::Matrix<float, 24, 1> x;

    // State covariance matrix
    std::unique_ptr<Eigen::Matrix<float, 24, 24>> Q;

    // State error covariance matrix
    std::unique_ptr<Eigen::Matrix<float, 24, 24>> P;

    // State transition matrix
    std::unique_ptr<Eigen::Matrix<float, 24, 24>> F;

    // Measurement noise covariance matrices

    Eigen::Matrix<float, 6, 6> R_ZERO_VEL;

    Eigen::Matrix<float, 3, 3> R_ACC;

    Eigen::Matrix<float, 3, 3> R_GYRO;

    // Zero velocity measurement matrix
    Eigen::Matrix<float, 6, 6> H_ZERO_VEL;

    // Acceleration measurement matrix
    Eigen::Matrix<float, 3, 6> H_ACC_GYRO;

    // Rotational matrix from body frame to inertial frame
    Eigen::Matrix<float, 3, 3> A_ROT;

    // On ramp quaternion
    Eigen::Vector4f onRampQuaternion;

    // NED gravity vector [m/s^2].
    const Eigen::Vector3f gravityNed{0.0f, 0.0f, -Constants::g};
};

}  // namespace Boardcore
