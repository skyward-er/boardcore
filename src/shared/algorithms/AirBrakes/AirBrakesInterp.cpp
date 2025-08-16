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

#include "AirBrakesInterp.h"

#include <logger/Logger.h>
#include <utils/Constants.h>

#include <limits>

#include "drivers/timer/TimestampTimer.h"
#include "utils/Debug.h"

namespace Boardcore
{

AirBrakesInterp::AirBrakesInterp(
    std::function<TimedTrajectoryPoint()> getCurrentPosition,
    const TrajectorySet& trajectoryOpenSet,
    const TrajectorySet& trajectoryCloseSet,
    const AirBrakesInterpConfig& configInterp,
    std::function<void(float)> setActuator)
    : getCurrentPosition(std::move(getCurrentPosition)),
      setActuator(std::move(setActuator)), trajectoryOpenSet(trajectoryOpenSet),
      trajectoryCloseSet(trajectoryCloseSet), configInterp(configInterp)
{
}

bool AirBrakesInterp::init() { return true; }

void AirBrakesInterp::begin(float currentMass)
{
    if (running)
        return;

    // Choose the best trajectories depending on the mass and the delta mass
    if (configInterp.DM == 0)
    {
        // Compatibility in case the mass information is not provided
        choosenOpenTrajectory  = &trajectoryOpenSet.trajectories[0];
        choosenCloseTrajectory = &trajectoryCloseSet.trajectories[0];
    }
    else
    {
        int index =
            round((currentMass - configInterp.INITIAL_MASS) / configInterp.DM);

        // Bound the index in order to have an indexable element
        index                 = std::max(index, 0);
        uint32_t boundedIndex = std::min(static_cast<uint32_t>(index),
                                         trajectoryOpenSet.length() - 1);

        choosenOpenTrajectory  = &trajectoryOpenSet.trajectories[boundedIndex];
        choosenCloseTrajectory = &trajectoryCloseSet.trajectories[boundedIndex];
    }

    Algorithm::begin();
}

void AirBrakesInterp::step()
{
    auto currentPosition = getCurrentPosition();

    // Do nothing if we have no new data
    if (lastPosition.timestamp >= currentPosition.timestamp)
        return;

    lastPosition = currentPosition;

    // Interpolation + PID
    float percentage = controlInterp(currentPosition);

    // Filtering
    float filterCoeff = 0;

    // If the altitude is lower than the minimum one, the filter is kept at the
    // same value, to avoid misleading filtering actions
    if (currentPosition.z < configInterp.FILTER_MINIMUM_ALTITUDE)
    {
        filterCoeff = configInterp.STARTING_FILTER_VALUE;
    }
    else
    {
        filterCoeff =
            configInterp.STARTING_FILTER_VALUE -
            (currentPosition.z - configInterp.FILTER_MINIMUM_ALTITUDE) *
                ((configInterp.STARTING_FILTER_VALUE) /
                 (configInterp.FILTER_MAXIMUM_ALTITUDE -
                  configInterp.FILTER_MINIMUM_ALTITUDE));
    }

    if (currentPosition.z < configInterp.ABK_CRITICAL_ALTITUDE)
    {
        // Compute the actual value filtered
        percentage =
            lastPercentage + (percentage - lastPercentage) * filterCoeff;
    }
    else
    {
        // If the height is beyond the target one, the algorithm tries to brake
        // as much as possible
        percentage = 1;
    }

    lastPercentage = percentage;
    setActuator(percentage);
}

float AirBrakesInterp::controlInterp(TrajectoryPoint currentPosition)
{
    // we take the index of the current point of the trajectory and we look
    // ahead of N points
    int index_z =
        floor((currentPosition.z / configInterp.DZ)) + configInterp.N_FORWARD;

    index_z = std::max(index_z, 0);

    // for safety we check whether the index exceeds the maximum index of the
    // trajectory sets
    index_z = std::min(index_z, std::min(choosenCloseTrajectory->trjSize - 1,
                                         choosenOpenTrajectory->trjSize - 1));

    Boardcore::TrajectoryPoint trjPointClosed =
        choosenCloseTrajectory->points[index_z];
    Boardcore::TrajectoryPoint trjPointOpen =
        choosenOpenTrajectory->points[index_z];

    // TRACE("z: %+.3f, curr: %+.3f, clos: %+.3f, open: %+.3f\n",
    //       currentPosition.z, currentPosition.vz, trjPointClosed.vz,
    //       trjPointOpen.vz);

    if (currentPosition.vz <= trjPointClosed.vz)
    {
        return 0;
    }
    else if (currentPosition.vz >= trjPointOpen.vz)
    {
        return 1;
    }
    else
    {
        float detectedPosition = (currentPosition.vz - trjPointClosed.vz) /
                                 (trjPointOpen.vz - trjPointClosed.vz);

        float dt = 1 / configInterp.ARB_FREQ;

        float ref       = configInterp.PID_REF;
        float error     = detectedPosition - ref;
        float prevError = lastPercentage - ref;

        if (saturation == false)
            integralError += error * dt;

        float kp = configInterp.PID_COEFFS[0];
        float ki = configInterp.PID_COEFFS[1];
        float kd = configInterp.PID_COEFFS[2];

        float percentage =
            kp * error + kd * prevError / dt + ki * integralError + ref;

        if (percentage < 0)
        {
            percentage = 0;
            saturation = true;
        }
        else if (percentage > 1)
        {
            percentage = 1;
            saturation = true;
        }
        else
            saturation = false;

        return percentage;
    }
}

}  // namespace Boardcore
