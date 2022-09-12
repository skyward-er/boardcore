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

#include "AirBrakes.h"

#include <logger/Logger.h>
#include <math.h>
#include <utils/Constants.h>

#include <limits>

#include "drivers/timer/TimestampTimer.h"
#include "utils/Debug.h"
using namespace std;

namespace Boardcore
{

AirBrakes::AirBrakes(function<TimedTrajectoryPoint()> getCurrentPosition,
                     const TrajectorySet &trajectorySet,
                     const AirBrakesConfig &config,
                     std::function<void(float)> setActuator,
                     bool interpAlgo = false)
    : getCurrentPosition(getCurrentPosition), trajectorySet(trajectorySet),
      config(config), setActuator(setActuator),
      pi(config.KP, config.KI, config.TS), interpAlgo(interpAlgo)
{
}

bool AirBrakes::init() { return true; }

void AirBrakes::begin()
{
    if (running)
        return;

    if (!interpAlgo)
    {
        lastPosition = getCurrentPosition();
        chooseTrajectory(lastPosition);
        TRACE("[ABK] EXECUTING PI\n");
    }
    else
    {
        filter_coeff = config.INITIAL_FILTER_COEFF;
        Tfilter      = config.INITIAL_T_FILTER;
        TRACE("[ABK] EXECUTING INTERP\n");
    }

    Algorithm::begin();
}

void AirBrakes::step()
{
    auto currentPosition = getCurrentPosition();

    // Do nothing if we have no new data
    if (lastPosition.timestamp >= currentPosition.timestamp)
        return;

    lastPosition = currentPosition;

    if (!interpAlgo)
    {

        auto setPoint = getSetpoint(currentPosition);
        float rho     = getRho(currentPosition.z);

        float targetDrag = piStep(currentPosition, setPoint, rho);
        float surface    = getSurface(currentPosition, rho, targetDrag);
        setActuator(surface / config.SURFACE);
    }
    else
    {
        // interpolation
        float percentage = controlInterp(currentPosition);
        // TRACE("perc: %f\n", percentage);

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

                // TRACE("reduced coefficient: %f\n", filter_coeff);
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
        // TRACE("perc_after: %f\n\n", percentage);
    }
}

void AirBrakes::setLiftoffTimestamp()
{
    this->tLiftoff = TimestampTimer::getTimestamp();
}

float AirBrakes::controlInterp(TrajectoryPoint currentPosition)
{
    float dz = 10;  // [TODO] take from PortugalTrajectorySet.h

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

void AirBrakes::chooseTrajectory(TrajectoryPoint currentPosition)
{
    float minDistance   = numeric_limits<float>::infinity();
    uint8_t trjIndexMin = trajectorySet.length() / 2;

    for (uint8_t trjIndex = 0; trjIndex < trajectorySet.length(); trjIndex++)
    {
        Trajectory &trajectory = trajectorySet.trajectories[trjIndex];

        for (uint32_t ptIndex = 0; ptIndex < trajectory.size(); ptIndex++)
        {
            TrajectoryPoint point = trajectory.points[ptIndex];
            float distance =
                TrajectoryPoint::distanceSquared(point, currentPosition);

            if (distance < minDistance)
            {
                minDistance            = distance;
                trjIndexMin            = trjIndex;
                lastSelectedPointIndex = ptIndex;
                chosenTrajectory       = &trajectory;
            }
        }
    }

    chosenTrajectory = &(trajectorySet.trajectories[trjIndexMin]);

    Logger::getInstance().log(AirBrakesChosenTrajectory{trjIndexMin});
}

TrajectoryPoint AirBrakes::getSetpoint(TrajectoryPoint currentPosition)
{
    if (chosenTrajectory == nullptr)
        return {};

    float minDistance = numeric_limits<float>::infinity();

    uint32_t end = chosenTrajectory->size();
    for (uint32_t ptIndex = lastSelectedPointIndex; ptIndex < end; ptIndex++)
    {
        float distanceFromCurrentInput =
            abs(chosenTrajectory->points[ptIndex].z - currentPosition.z);

        if (distanceFromCurrentInput < minDistance)
        {
            minDistance            = distanceFromCurrentInput;
            lastSelectedPointIndex = ptIndex;
        }
    }

    return chosenTrajectory->points[lastSelectedPointIndex];
}

float AirBrakes::getRho(float z)
{
    return Constants::RHO_0 * expf(-z / Constants::Hn);
}

float AirBrakes::piStep(TimedTrajectoryPoint currentPosition,
                        TrajectoryPoint setPoint, float rho)
{
    const float cdMin   = getCD(currentPosition, 0);
    const float dragMin = getDrag(currentPosition, cdMin, rho);

    const float cdMax   = getCD(currentPosition, config.EXTENSION);
    const float dragMax = getDrag(currentPosition, cdMax, rho);

    // Get target surface percentage
    const float cdRef   = getCD(currentPosition, chosenTrajectory->extension);
    const float dragRef = getDrag(currentPosition, cdRef, rho);

    // PI update
    const float error = currentPosition.vz - setPoint.vz;
    const float dragPi =
        pi.antiWindUp(pi.update(error) + dragRef, dragMin, dragMax);

    return dragPi;
}

float AirBrakes::getSurface(const TimedTrajectoryPoint &currentPosition,
                            float rho, float targetDrag)
{
    float bestDDrag   = numeric_limits<float>::infinity();
    float bestSurface = 0;

    // TODO: Drags are monotone, here the algorithm can be more efficient
    for (uint8_t step = 0; step < config.DRAG_STEPS; step++)
    {
        float surface = (step / (config.DRAG_STEPS - 1)) * config.SURFACE;

        float extension = getExtension(surface);
        float cd        = getCD(currentPosition, extension);
        float drag      = getDrag(currentPosition, cd, rho);
        float dDrag     = abs(targetDrag - drag);

        if (dDrag < bestDDrag)
        {
            bestDDrag   = dDrag;
            bestSurface = surface;
        }
    }

    return bestSurface;
}

float AirBrakes::getExtension(float surface)
{
    // clang-format off
    return
        config.EXT_POL_1 * powf(surface, 4) +
        config.EXT_POL_2 * powf(surface, 3) +
        config.EXT_POL_3 * powf(surface, 2) +
        config.EXT_POL_4 * surface;
    // clang-format on
}

float AirBrakes::getCD(TimedTrajectoryPoint currentPosition, float extension)
{
    const float mach1 = currentPosition.getMac();
    const float mach2 = powf(mach1, 2);
    const float mach3 = powf(mach1, 3);
    const float mach4 = powf(mach1, 4);
    const float mach5 = powf(mach1, 5);
    const float mach6 = powf(mach1, 6);

    const float extension2 = powf(extension, 2);

    // clang-format off
    return
        config.N000 +
        config.N100 * mach1 +
        config.N200 * mach2 +
        config.N300 * mach3 +
        config.N400 * mach4 +
        config.N500 * mach5 +
        config.N600 * mach6 +
        config.N010 * extension +
        config.N020 * extension2 +
        config.N110 * extension  * mach1 +
        config.N120 * extension2 * mach1 +
        config.N210 * extension  * mach2 +
        config.N220 * extension2 * mach2 +
        config.N310 * extension  * mach3 +
        config.N320 * extension2 * mach3 +
        config.N410 * extension  * mach4 +
        config.N420 * extension2 * mach4 +
        config.N510 * extension  * mach5 +
        config.N520 * extension2 * mach5 +
        config.N001 * currentPosition.z;
    // clang-format on
}

float AirBrakes::getDrag(TimedTrajectoryPoint currentPosition, float cd,
                         float rho)
{
    return 0.5 * rho * config.S0 * cd * currentPosition.vz *
           currentPosition.vMod;
}

}  // namespace Boardcore
