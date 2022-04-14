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

#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

#include <Eigen/Dense>

#include "ExtendedKalmanConfig.h"

namespace Boardcore
{

class ExtendedKalman
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

    explicit ExtendedKalman(ExtendedKalmanConfig config);

    /**
     * @brief Prediction with accelerometer data.
     *
     * @param u Vector with acceleration data [x y z][m/s^2]
     */
    void predictAcc(const Eigen::Vector3f& acceleration);

    /**
     * @brief Prediction with gyroscope data.
     *
     * @param u Vector with angular velocity data [x y z]
     */
    void predictGyro(const Eigen::Vector3f& angularVelocity);

    /**
     * @brief Correction with barometer data.
     *
     * @param pressure Pressure read from the barometer [Pa]
     * @param mslPress Pressure at sea level [Pa]
     * @param mslTemp Temperature at sea level [Kelvin]
     */
    void correctBaro(const float pressure, const float mslPress,
                     const float mslTemp);

    /**
     * @brief Correction with gps data.
     *
     * @param y 4x1 Vector of the gps readings [n e vn ve][m m m/s m/s]
     * @param sats_num Number of satellites available
     */
    void correctGPS(const Eigen::Vector4f& gps, const uint8_t sats_num);

    /**
     * @brief MEKF correction of the magnetometer readings.
     *
     * @param y 3x1 Normalized vector of the magnetometer readings [x y z].
     */
    void correctMag(const Eigen::Vector3f& y);

    /**
     * @return 13x1 State vector [n e d vn ve vd qx qy qz qw bx by bz].
     */
    Eigen::Matrix<float, 13, 1> getState() const;

    /**
     * @param x 13x1 State vector [n e d vn ve vd qx qy qz qw bx by bz].
     */
    void setX(const Eigen::Matrix<float, 13, 1>& x);

private:
    /**
     * @brief Return a rotation matrix from body from to NED frame;
     *
     * TODO: Move to SkyQuaternion utilities
     */
    Eigen::Matrix3f body2ned(const Eigen::Vector4f& q);

    ///< Extended Kalman filter configuration parameters
    ExtendedKalmanConfig config;

    ///< State vector [n e d vn ve vd qx qy qz qw bx by bz]
    Eigen::Matrix<float, 13, 1> x;

    ///< TODO
    Eigen::Matrix<float, 12, 12> P;

    ///< NED gravity vector [m/s^2]
    Eigen::Vector3f gravityNed{0.0f, 0.0f, Boardcore::Constants::g};

    // Utility matrix used for the accelerometer
    Eigen::Matrix<float, 6, 6> Q_lin;

    // Utility matrixes used for the gps
    Eigen::Matrix<float, 4, 6> H_gps;
    Eigen::Matrix<float, 6, 4> H_gps_tr;
    Eigen::Matrix<float, 4, 4> R_gps;

    // Utility matrix used for the magnetometer
    Eigen::Matrix3f R_mag;
    Eigen::Matrix<float, 6, 6> Q_mag;

    // Other utility matrixes
    Eigen::Matrix<float, 6, 6> F;
    Eigen::Matrix<float, 6, 6> Ftr;
    Eigen::Matrix<float, 6, 6> Fatt;
    Eigen::Matrix<float, 6, 6> Fatttr;
    Eigen::Matrix<float, 6, 6> Gatt;
    Eigen::Matrix<float, 6, 6> Gatttr;
    Eigen::Matrix<float, 6, 6> Patt;
};

}  // namespace Boardcore
