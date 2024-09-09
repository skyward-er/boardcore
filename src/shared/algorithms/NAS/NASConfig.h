/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace Boardcore
{

struct NASConfig
{
    float T;                    ///< [s]       Sample period
    float SIGMA_BETA;           ///< [rad/s^2] Estimated gyroscope bias variance
    float SIGMA_W;              ///< [rad]   Estimated gyroscope variance
    float SIGMA_ACC;            ///< [m/s^2]    Estimated accelerometer variance
    float SIGMA_MAG;            ///< [uT]    Estimated magnetometer variance
    Eigen::Vector4f SIGMA_GPS;  ///< [millideg, millideg, m/s, m/s]
                                ///< estimated GPS variance. position from test,
                                ///< velocity from datasheet
    float SIGMA_BAR;            ///< [Pa]     Estimated altitude variance
    float SIGMA_POS;    ///< [m]     Estimated variance of the position noise
    float SIGMA_VEL;    ///< [m/s] Estimated variance of the velocity noise
    float SIGMA_PITOT;  ///< [Pa]    Estimated variance of the pitot velocity

    float P_POS;           ///< Position prediction covariance horizontal
    float P_POS_VERTICAL;  ///< Position prediction covariance vertical

    float P_VEL;           ///< Velocity prediction covariance horizontal
    float P_VEL_VERTICAL;  ///< Velocity prediction covariance vertical

    float P_ATT;   ///< Attitude prediction covariance
    float P_BIAS;  ///< Bias prediction covariance

    float SATS_NUM = 6.0f;  ///< Number of satellites used at setup time

    Eigen::Vector3f NED_MAG;  ///< Normalized magnetic field vector in NED frame
};

}  // namespace Boardcore
