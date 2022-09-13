/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco, Alberto Nidasio, Emilio Corigliano
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

#include <actuators/Servo/Servo.h>
#include <algorithms/PIController.h>

#include <functional>
#include <vector>

#include "AirBrakes.h"
#include "AirBrakesData.h"
#include "AirBrakesInterpConfig.h"
#include "TrajectorySet.h"

namespace Boardcore
{

class AirBrakesInterp : public AirBrakes
{
public:
    AirBrakesInterp(std::function<TimedTrajectoryPoint()> getCurrentPosition,
                    const TrajectorySet &trajectorySet,
                    const AirBrakesInterpConfig &config,
                    std::function<void(float)> setActuator, float dz);

    bool init() override;

    /**
     * @brief This method chooses the trajectory the rocket will follow and
     * starts the algorithm.
     */
    void begin();

    /**
     * @brief Looks for nearest point in the current chosen trajectory and moves
     * the airbraks according to the current rocket speed and the prediction.
     */
    void step() override;

    /**
     * @brief registers the timestamp of liftoff
     */
    void setLiftoffTimestamp();

private:
    /**
     * @brief Calculates the percentage of aperture of the airbrakes
     * interpolating the trajectory points of the fully closed and fully opened
     * references
     */
    float controlInterp(TrajectoryPoint currentPosition);

    const AirBrakesInterpConfig &config;  ///< specialized config for interp
    uint64_t tLiftoff;                    ///< timestamp of the liftoff
    float lastPercentage;                 ///< last opening of the airbrakes
    float filter_coeff;   ///< how much the new aperture impacts the real one
    float Tfilter;        ///< [s] time from liftoff when to update filter
    bool filter = false;  ///< whether to apply the filter or not
    float dz;  ///< [m] the distance between two consecutive Trajectory points
};

}  // namespace Boardcore
