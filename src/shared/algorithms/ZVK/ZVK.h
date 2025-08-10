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

 #include <Eigen/Dense>
 #include <sensors/SensorData.h>
 #include <utils/Constants.h>
 #include "ZVKConfig.h"
 #include "ZVKState.h"
 

namespace Boardcore 
{

class ZVK
{

public:
    // Index of quaternions elements in the state.
    static constexpr uint16_t IDX_QUAT = 0;

    // Index of velocity elements in the state.
    static constexpr uint16_t IDX_VEL = 4;

    // Index of position elements in the state.
    static constexpr uint16_t IDX_POS = 7;

    // Index of accelerometer bias elements in the state.
    static constexpr uint16_t IDX_BIAS_ACC = 10;

    // Index of gyroscope bias elements in the state.
    static constexpr uint16_t IDX_BIAS_GYRO = 13;

    explicit ZVK(const ZVKConfig& config);

    /**
     * @brief Prediction.
     *
     * @param acceleration Vector with acceleration data [x y z][m/s^2].
     * @param angularSpeed Vector with angular velocity data [x y z][rad/s].

     */
    void predict(const Eigen::Vector3f& acceleration, const Eigen::Vector3f& angularSpeed);

    /**
     * @brief Prediction.
     *
     * @param acceleration Accelerometer data [m/s^2].
     * @param angularSpeed Gyroscope data [rad/s].
     */
    void predict(const AccelerometerData& acceleration, const GyroscopeData& angularSpeed);

    /**
     * @brief Correction.
     *
     * @param acceleration Vector with acceleration data [x y z][m/s^2].
     * @param angularSpeed Vector with angular velocity data [x y z][rad/s].
     * @param mag Normalized vector of the magnetometer readings [x y z][uT].

     */
    void correct(const Eigen::Vector3f& acceleration, const Eigen::Vector3f& angularSpeed, const Eigen::Vector3f& mag);

    /**
     * @brief Correction.
     *
     * @param acceleration Accelerometer data [m/s^2].
     * @param angularSpeed Gyroscope data [rad/s].
     * @param mag Magnetometer data [uT] 
     */
    void correct(const AccelerometerData& acceleration, const GyroscopeData& angularSpeed, const MagnetometerData& mag);

    /**
     * @return EKF state.
     */
    ZVKState getState() const;

    /**
     * @return State vector [qx qy qz qw vn ve vd n e d bax bay baz bgx bgy bgz].
     */
    Eigen::Matrix<float, 16, 1> getX() const;

    /**
     * @param state State vector [qx qy qz qw vn ve vd n e d bax bay baz bgx bgy bgz].
     */
    void setX(const Eigen::Matrix<float, 16, 1>& x);

    /**
     * @param state EKF state.
     */
    void setState(const ZVKState& state);



private:

    // Extended kalman filter configuration parameters
    ZVKConfig config;

    // State vector [qx, qy, qw, qz, vn, ve, vd, n, e, d, bax, bay, baz, bgx, bgy, bgz]
    Eigen::Matrix<float,16,1> x;

    // State covariance matrix
    Eigen::Matrix<float, 15, 15> P;

    // Measurement noise covariance matrix
    Eigen::Matrix<float, 12, 12> R;

    // Process noise covariance matrix 
    Eigen::Matrix<float, 12, 12> Q;    

    // NED gravity vector [m/s^2].
    const Eigen::Vector3f gravityNed{0.0f, 0.0f, -Constants::g};

    // Small angle error constant needed for the definition of matrix P
    float const smallAngleError = 0.08726646;

    // Constant needed for the definition of the definition of matrix R 
    float const rConst = 1e-6;



};

 } // namespace Boardcore