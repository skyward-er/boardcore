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

#include "AirBrakesPI.h"

#include <logger/Logger.h>
#include <math.h>
#include <utils/Constants.h>

#include <limits>

#include "drivers/timer/TimestampTimer.h"
#include "utils/Debug.h"
using namespace std;

namespace Boardcore
{

AirBrakesPI::AirBrakesPI(
    std::function<TimedTrajectoryPoint()> getCurrentPosition,
    const TrajectorySet &trajectorySet, const AirBrakesConfig &config,
    const AirBrakesPIConfig &configPI, std::function<void(float)> setActuator)
    : AirBrakes(getCurrentPosition, config, setActuator),
      pi(configPI.KP, configPI.KI, configPI.TS), trajectorySet(trajectorySet)
{
}

bool AirBrakesPI::init() { return true; }

void AirBrakesPI::begin()
{
    if (running)
        return;

    lastPosition = getCurrentPosition();
    chooseTrajectory(lastPosition);
    Algorithm::begin();
}

void AirBrakesPI::step()
{
    auto currentPosition = getCurrentPosition();

    // Do nothing if we have no new data
    if (lastPosition.timestamp >= currentPosition.timestamp)
        return;

    lastPosition = currentPosition;

    auto setPoint = getSetpoint(currentPosition);
    float rho     = getRho(currentPosition.z);

    float targetDrag = piStep(currentPosition, setPoint, rho);
    float surface    = getSurface(currentPosition, rho, targetDrag);
    setActuator(surface / config.SURFACE);
}

void AirBrakesPI::chooseTrajectory(TrajectoryPoint currentPosition)
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

TrajectoryPoint AirBrakesPI::getSetpoint(TrajectoryPoint currentPosition)
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

float AirBrakesPI::piStep(TimedTrajectoryPoint currentPosition,
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

}  // namespace Boardcore
