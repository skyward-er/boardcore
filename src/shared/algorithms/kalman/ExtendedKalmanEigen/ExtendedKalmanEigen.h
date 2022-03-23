/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include <math/SkyQuaternion.h>
#include <utils/aero/AeroUtils.h>

#include <Eigen/Dense>

#include "ExtendedKalmanConfig.h"

namespace Boardcore
{

using VectorNf = Eigen::Matrix<float, 13, 1>;

class ExtendedKalmanEigen
{

public:
    explicit ExtendedKalmanEigen(ExtendedKalmanConfig config);

    /**
     * @brief Prediction step of the EKF.
     *
     * @param u 3x1 Vector of the accelerometer readings [ax ay az].
     */
    void predict(const Eigen::Vector3f& u);

    /**
     * @brief EKF correction of the barometer data.
     *
     * @param y Pressure read from the barometer [Pa]
     * @param msl_press Pressure at sea level [Pa]
     * @param msl_temp Temperature at sea level [Kelvin]
     */
    void correctBaro(const float y, const float msl_press,
                     const float msl_temp);

    /**
     * @brief EKF correction of the gps readings.
     *
     * @param y 4x1 Vector of the gps readings [longitude, latitude,
     * gps_nord_vel, gps_east_vel].
     * @param sats_num Number of satellites available
     */
    void correctGPS(const Eigen::Vector4f& y, const uint8_t sats_num);

    /**
     * @brief Prediction step of the Multiplicative EKF.
     *
     * @param u 3x1 Vector of the gyroscope readings [wx wy wz].
     */
    void predictMEKF(const Eigen::Vector3f& u);

    /**
     * @brief MEKF correction of the magnetometer readings.
     *
     * @param y 3x1 Vector of the magnetometer readings [mx my mz].
     */
    void correctMEKF(const Eigen::Vector3f& y);

    /**
     * @return 13x1 State vector [px py pz vx vy vz qx qy qz qw bx by bz].
     */
    const VectorNf& getState();

    /**
     * @param x 13x1 State vector [px py pz vx vy vz qx qy qz qw bx by bz].
     */
    void setX(const VectorNf& x);

private:
    // Dimensions of matrices and vectors

    static constexpr uint16_t N = 13;  ///< State vector elements, N x 1

    /**
     * @brief P matrix, N-1 x N-1.
     *
     * Reduced order thanks to the MEKF.
     */
    static constexpr uint16_t NP = N - 1;

    /**
     * @brief Number of attitude related elements.
     *
     * Quaternion components and biases: [q1, q2, q3, q4, bx, by, bz]
     */
    static constexpr uint16_t NATT = 7;

    /**
     * @brief Number of linear elements in the state vector.
     *
     * Position and velocity: [p_north, p_east, p_down, v_n, v_e, v_d]
     */
    static constexpr uint16_t NL = N - NATT;

    ///< States of the barometer [pressure]
    static constexpr uint16_t NBAR = 1;

    ///< States of the gps [lon, lat, v_north, v_east]
    static constexpr uint16_t NGPS = 4;

    ///< States of the magnetometer [mx, my, mz]
    static constexpr uint16_t NMAG = 3;

    /**
     * @brief Dimension used in the MEKF.
     *
     * 4 quaternion components + 3 biases - 1.
     * The MEKF structure allows us to perform the dimensionality reduction of 1
     */
    static constexpr uint16_t NMEKF = 6;

    ///< Extended Kalman filter configuration parameters
    ExtendedKalmanConfig config;

    VectorNf x;
    Eigen::Matrix<float, NP, NP> P;
    Eigen::Matrix<float, NL, NL> F;
    Eigen::Matrix<float, NL, NL> Ftr;

    Eigen::Matrix3f P_pos;
    Eigen::Matrix3f P_vel;
    Eigen::Matrix3f P_att;
    Eigen::Matrix3f P_bias;
    Eigen::Matrix<float, NL, NL> Plin;

    Eigen::Matrix3f Q_pos;
    Eigen::Matrix3f Q_vel;
    Eigen::Matrix<float, NL, NL> Q_lin;

    Eigen::Vector3f g;
    Eigen::Matrix2f eye2;
    Eigen::Matrix3f eye3;
    Eigen::Matrix4f eye4;
    Eigen::Matrix<float, 6, 6> eye6;

    Eigen::Matrix<float, NBAR, NBAR> R_bar;

    Eigen::Matrix<float, NGPS, NGPS> R_gps;
    Eigen::Matrix<float, NGPS, NL> H_gps;
    Eigen::Matrix<float, NL, NGPS> H_gpstr;

    Eigen::Vector4f q;
    Eigen::Matrix<float, NMAG, NMAG> R_mag;
    Eigen::Matrix<float, NMEKF, NMEKF> Q_mag;
    Eigen::Matrix<float, NMEKF, NMEKF> Fatt;
    Eigen::Matrix<float, NMEKF, NMEKF> Fatttr;
    Eigen::Matrix<float, NMEKF, NMEKF> Gatt;
    Eigen::Matrix<float, NMEKF, NMEKF> Gatttr;
    Eigen::Matrix<float, NMEKF, NMEKF> Patt;

    Boardcore::SkyQuaternion quater;
};

}  // namespace Boardcore
