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

#include <algorithms/Algorithm.h>

#include <functional>
#include <vector>

#include "AirBrakesInterpConfig.h"
#include "TrajectorySet.h"

namespace Boardcore
{

class AirBrakesInterp : public Algorithm
{
public:
    AirBrakesInterp(std::function<TimedTrajectoryPoint()> getCurrentPosition,
                    const TrajectorySet &trajectoryOpenSet,
                    const TrajectorySet &trajectoryCloseSet,
                    const AirBrakesInterpConfig &configInterp,
                    std::function<void(float)> setActuator);

    bool init() override;

    /**
     * @brief This method chooses the trajectory set that will be used to
     * control the algorithm and starts it.
     *
     * @param currentMass The current estimated rocket mass.
     */
    void begin(float currentMass);

private:
    /**
     * @brief Looks for nearest point in the current chosen trajectory and moves
     * the airbrakes according to the current rocket speed and the prediction.
     */
    void step() override;

    /**
     * @brief Calculates the percentage of aperture of the airbrakes
     * interpolating the trajectory points of the fully closed and fully opened
     * references
     */
    float controlInterp(TrajectoryPoint currentPosition);

    std::function<TimedTrajectoryPoint()> getCurrentPosition;
    std::function<void(float)> setActuator;

    TimedTrajectoryPoint lastPosition;

    // Trajectory sets (open and closed) from which the algorithm will choose at
    // the beginning the tuple with which interpolate the data. The selection
    // depends on the rocket mass.
    const TrajectorySet &trajectoryOpenSet;
    const TrajectorySet &trajectoryCloseSet;

    // Choosen trajectories from the begin function
    Trajectory *choosenCloseTrajectory = nullptr;
    Trajectory *choosenOpenTrajectory  = nullptr;

    AirBrakesInterpConfig configInterp;  ///< specialized config
    float lastPercentage = 0;            ///< last opening of the airbrakes
};

}  // namespace Boardcore
