/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Marco Cella, Alberto Nidasio
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

#include "NAS.h"

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
     *   https://en.wikipedia.org/wiki/Triad_method
     *   https://www.aero.iitb.ac.in/satelliteWiki/index.php/Triad_Algorithm
     *
     * @param acc Normalized accelerometer readings [x y z].
     * @param mag Normalized magnetometer readings [x y z].
     * @param nedMag Normalized magnetic field vector in NED frame [x y z].
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
}  // namespace Boardcore
