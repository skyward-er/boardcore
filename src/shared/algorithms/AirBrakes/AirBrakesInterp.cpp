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
    const TrajectorySet &trajectorySet, const AirBrakesInterpConfig &config,
    std::function<void(float)> setActuator, float dz)
    : AirBrakes(getCurrentPosition, trajectorySet, config, setActuator),
      config(config), tLiftoff(-1), lastPercentage(0),
      filter_coeff(config.INITIAL_FILTER_COEFF),
      Tfilter(config.INITIAL_T_FILTER), filter(false), dz(dz)
{
}

bool AirBrakesInterp::init() { return true; }

void AirBrakesInterp::begin()
{
    if (running)
        return;

    filter_coeff = config.INITIAL_FILTER_COEFF;
    Tfilter      = config.INITIAL_T_FILTER;
    TRACE("[ABK] EXECUTING INTERP\n");

    Algorithm::begin();
}

void AirBrakesInterp::step()
{
    auto currentPosition = getCurrentPosition();

    // Do nothing if we have no new data
    if (lastPosition.timestamp >= currentPosition.timestamp)
        return;

    lastPosition = currentPosition;

    // interpolation
    float percentage = controlInterp(currentPosition);

    // filter the aperture value only from the second step
    if (filter)
    {
        percentage =
            lastPercentage + (percentage - lastPercentage) * filter_coeff;

        // if the time elapsed from liftoff is greater or equal than the
        // Tfilter (converted in microseconds as for the timestamp), we
        // update the filter
        uint64_t currentTimestamp = TimestampTimer::getTimestamp();
        if (currentTimestamp - tLiftoff >= Tfilter * 1e6)
        {
            Tfilter += config.DELTA_T_FILTER;
            filter_coeff /= config.FILTER_RATIO;
        }
    }
    else
    {
        TRACE("START FILTERING\n");
        TRACE("tLiftoff: %llu\n", tLiftoff);
        filter = true;
    }

    lastPercentage = percentage;
    setActuator(percentage);
}

void AirBrakesInterp::setLiftoffTimestamp()
{
    this->tLiftoff = TimestampTimer::getTimestamp();
}

float AirBrakesInterp::controlInterp(TrajectoryPoint currentPosition)
{
    // we take the index of the current point of the trajectory and we look
    // ahead of 2 points
    int index_z = floor(currentPosition.z / dz) + 2;

    Boardcore::Trajectory t_closed = trajectorySet.trajectories[0];
    Boardcore::Trajectory t_opened =
        trajectorySet.trajectories[trajectorySet.trjSize - 1];

    // for safety we check whether the index exceeds the maximum index of the
    // trajectory sets
    index_z =
        std::min(index_z, std::min(t_closed.trjSize - 1, t_opened.trjSize - 1));

    Boardcore::TrajectoryPoint trjPointClosed = t_closed.points[index_z];
    Boardcore::TrajectoryPoint trjPointOpen   = t_opened.points[index_z];

    // TRACE("z: %+.3f, curr: %+.3f, clos: %+.3f, open: %+.3f\n",
    //       currentPosition.z, currentPosition.vz, trjPointClosed.vz,
    //       trjPointOpen.vz);

    if (currentPosition.vz <= trjPointClosed.vz)
        return 0;
    else if (currentPosition.vz >= trjPointOpen.vz)
        return 1;
    else
        return (currentPosition.vz - trjPointClosed.vz) /
               (trjPointOpen.vz - trjPointClosed.vz);
}

}  // namespace Boardcore
