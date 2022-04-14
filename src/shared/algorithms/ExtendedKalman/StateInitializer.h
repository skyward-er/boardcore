/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Marco Cella, Alberto Nidasio
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

#include <diagnostic/PrintLogger.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <Eigen/Dense>
#include <iostream>

#include "ExtendedKalman.h"

namespace Boardcore
{

/**
 * @brief Utility used to initialize the extended kalman filter's state.
 *
 * The intended use is the following:
 * - Instantiate the object, the state is initialize to zero;
 * - Call positionInit to set the initial position;
 * - Call either eCompass or triad to set the initial orientation;
 * - Get the initial state for the kalman with getInitX
 */
class StateInitializer
{
public:
    /**
     * @brief Initialize the state of the Extended Kalman Filter.
     *
     * The state is initialized to zero. Velocity should be null and gyro biases
     * can be left to zero since the sensor performs self-calibration and the
     * measurements are already compensated.
     */
    StateInitializer();

    /**
     * @brief Ecompass algorithm to estimate the attitude before the liftoff.
     *
     * ecompass reference:
     *   https://de.mathworks.com/help/fusion/ref/ecompass.html
     *
     * @param acc 3x1 accelerometer readings [x y z][m/s^2].
     * @param mag 3x1 magnetometer readings [x y z][uT].
     *
     */
    void eCompass(const Eigen::Vector3f acc, const Eigen::Vector3f mag);

    /**
     * @brief Triad algorithm to estimate the attitude before the liftoff.
     *
     * triad reference:
     *   https://www.aero.iitb.ac.in/satelliteWiki/index.php/Triad_Algorithm
     *
     * @param acc 3x1 accelerometer readings [x y z][m/s^2].
     * @param mag 3x1 magnetometer readings [x y z][uT].
     * @param nedMag 3x1 magnetometer readings [x y z][uT].
     */
    void triad(Eigen::Vector3f& acc, Eigen::Vector3f& mag,
               Eigen::Vector3f& nedMag);

    /**
     * @brief Initialization of the position at a specific altitude
     *
     * @param pressure Current pressure [Pas]
     * @param pressureRef Pressure at reference altitude (must be > 0) [Pa]
     * @param temperatureRef Temperature at reference altitude [K]
     */
    void positionInit(const float pressure, const float pressureRef,
                      const float temperatureRef);

    Eigen::Matrix<float, 13, 1> getInitX();

private:
    Eigen::Matrix<float, 13, 1> x_init;

    PrintLogger log = Logging::getLogger("initstates");
};

StateInitializer::StateInitializer() { x_init << Eigen::MatrixXf::Zero(13, 1); }

void StateInitializer::eCompass(const Eigen::Vector3f acc,
                                const Eigen::Vector3f mag)
{
    // ndr: since this method runs only when the rocket is stationary, there's
    // no need to add the gravity vector because the accelerometers already
    // measure it. This is not true if we consider the flying rocket.

    Eigen::Vector3f am(acc.cross(mag));

    Eigen::Matrix3f R;
    R << am.cross(acc), am, acc;
    R.col(0).normalize();
    R.col(1).normalize();
    R.col(2).normalize();

    Eigen::Vector4f x_quat = SkyQuaternion::rotationMatrix2quat(R);

    x_init(ExtendedKalman::IDX_QUAT)     = x_quat(0);
    x_init(ExtendedKalman::IDX_QUAT + 1) = x_quat(1);
    x_init(ExtendedKalman::IDX_QUAT + 2) = x_quat(2);
    x_init(ExtendedKalman::IDX_QUAT + 3) = x_quat(3);
}

void StateInitializer::triad(Eigen::Vector3f& acc, Eigen::Vector3f& mag,
                             Eigen::Vector3f& nedMag)
{
    // Prepare the gravity vector in NED frame
    Eigen::Vector3f gravityNed(0.0F, 0.0F, -Constants::g);

    Eigen::Vector3f R1 = gravityNed;
    R1.normalize();
    Eigen::Vector3f R2 = gravityNed.cross(nedMag);
    R2.normalize();
    Eigen::Vector3f R3 = R1.cross(R2);

    Eigen::Vector3f r1 = acc;
    r1.normalize();
    Eigen::Vector3f r2 = acc.cross(mag);
    r2.normalize();
    Eigen::Vector3f r3 = r1.cross(r2);

    Eigen::Matrix3f Mr;
    Mr << R1, R2, R3;

    Eigen::Matrix3f Mou;
    Mou << r1, r2, r3;

    Eigen::Matrix3f dcm = Mr * Mou.transpose();

    Eigen::Vector4f q = SkyQuaternion::rotationMatrix2quat(dcm);

    x_init(ExtendedKalman::IDX_QUAT)     = q(0);
    x_init(ExtendedKalman::IDX_QUAT + 1) = q(1);
    x_init(ExtendedKalman::IDX_QUAT + 2) = q(2);
    x_init(ExtendedKalman::IDX_QUAT + 3) = q(3);
}

void StateInitializer::positionInit(const float pressure,
                                    const float pressureRef,
                                    const float temperatureRef)
{
    x_init(0) = 0.0;
    x_init(1) = 0.0;
    // msl altitude (in NED, so negative)
    x_init(2) = -Aeroutils::relAltitude(pressure, pressureRef, temperatureRef);
}

Eigen::Matrix<float, 13, 1> StateInitializer::getInitX() { return x_init; }

}  // namespace Boardcore
