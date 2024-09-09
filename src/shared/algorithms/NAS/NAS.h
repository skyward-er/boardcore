/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
 * Authors: Alessandro Del Duca, Luca Conterio, Marco Cella
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

#include <algorithms/ReferenceValues.h>
#include <sensors/SensorData.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

#include <Eigen/Dense>

#include "NASConfig.h"
#include "NASState.h"

namespace Boardcore
{

class NAS
{
public:
    ///< Index of position elements in the state.
    static constexpr uint16_t IDX_POS = 0;

    ///< Index of velocity elements in the state.
    static constexpr uint16_t IDX_VEL = 3;

    ///< Index of quaternions elements in the state.
    static constexpr uint16_t IDX_QUAT = 6;

    ///< Index of bias elements in the state.
    static constexpr uint16_t IDX_BIAS = 10;

    explicit NAS(NASConfig config);

    /**
     * @brief Prediction with accelerometer data.
     *
     * @param acceleration Vector with acceleration data [x y z][m/s^2].
     */
    void predictAcc(const Eigen::Vector3f& acceleration);

    /**
     * @brief Prediction with accelerometer data.
     *
     * @param acceleration Accelerometer data [m/s^2].
     */
    void predictAcc(const AccelerometerData& acceleration);

    /**
     * @brief Prediction with gyroscope data.
     *
     * @param angularSpeed Vector with angular velocity data [x y z][rad/s].
     */
    void predictGyro(const Eigen::Vector3f& angularSpeed);

    /**
     * @brief Prediction with gyroscope data.
     *
     * @param angularSpeed Gyroscope data [rad/s].
     */
    void predictGyro(const GyroscopeData& angularSpeed);

    /**
     * @brief Correction with barometer data.
     *
     * @param pressure Pressure read from the barometer [Pa].
     */
    void correctBaro(const float pressure);

    /**
     * @brief Correction with gps data.
     *
     * @param gps Vector of the gps readings [n e vn ve][m m m/s m/s].
     */
    void correctGPS(const Eigen::Vector4f& gps);

    /**
     * @brief Correction with gps data only if fix is acquired.
     *
     * @param gps GPS data.
     */
    void correctGPS(const GPSData& gps);

    /**
     * @brief Correction with magnetometer data.
     *
     * @param mag Normalized vector of the magnetometer readings [x y z][uT].
     */
    void correctMag(const Eigen::Vector3f& mag);

    /**
     * @brief Correction with magnetometer data.
     *
     * @param mag Magnetometer data [uT].
     */
    void correctMag(const MagnetometerData& mag);

    /**
     * @brief Correction with accelerometer data.
     *
     * @param u Normalized vector with acceleration data [x y z][m/s^2].
     */
    void correctAcc(const Eigen::Vector3f& acc);

    /**
     * @brief Correction with accelerometer data.
     *
     * @param u Acceleration data [m/s^2].
     */
    void correctAcc(const AccelerometerData& acc);

    /**
     * @brief Correction with pitot pressures.
     *
     * @warning Do not call this function after apogee!
     *
     * @param staticPressure Is the static pressure measured from the pitot
     * sensor.
     * @param dynamicPressure Is the dynamic pressure measured from the pitot
     * sensor (difference pressure between total pressure and static pressure).
     */
    void correctPitot(const float staticPressure, const float dynamicPressure);

    /**
     * @return EKF state.
     */
    NASState getState() const;

    /**
     * @return State vector [n e d vn ve vd qx qy qz qw bx by bz].
     */
    Eigen::Matrix<float, 13, 1> getX() const;

    /**
     * @param state EKF state.
     */
    void setState(const NASState& state);

    /**
     * @param state State vector [n e d vn ve vd qx qy qz qw bx by bz].
     */
    void setX(const Eigen::Matrix<float, 13, 1>& x);

    /**
     * @brief Changes the reference values.
     */
    void setReferenceValues(const ReferenceValues reference);

    /**
     * @brief Returns the current reference values.
     */
    ReferenceValues getReferenceValues();

private:
    ///< Extended Kalman filter configuration parameters.
    NASConfig config;

    ///< Reference values used for altitude and gps position.
    ReferenceValues reference;

    ///< State vector [n e d vn ve vd qx qy qz qw bx by bz].
    Eigen::Matrix<float, 13, 1> x;

    ///< Covariance matrix.
    Eigen::Matrix<float, 12, 12> P;

    ///< NED gravity vector [m/s^2].
    const Eigen::Vector3f gravityNed{0.0f, 0.0f, -Constants::g};

    // Utility matrixes used for the gps
    Eigen::Matrix<float, 4, 6> H_gps;
    Eigen::Matrix<float, 6, 4> H_gps_tr;
    Eigen::Matrix<float, 4, 4> R_gps;

    // Utility matrixes
    Eigen::Matrix3f R_acc;
    Eigen::Matrix3f R_mag;
    Eigen::Matrix<float, 6, 6> Q_quat;
    Eigen::Matrix<float, 6, 6> Q_lin;
    Eigen::Matrix<float, 6, 6> F_lin;
    Eigen::Matrix<float, 6, 6> F_lin_tr;
    Eigen::Matrix<float, 6, 6> F_quat;
    Eigen::Matrix<float, 6, 6> F_quat_tr;
    Eigen::Matrix<float, 6, 6> G_quat;
    Eigen::Matrix<float, 6, 6> G_quat_tr;
};

}  // namespace Boardcore
