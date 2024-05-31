/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Niccolò Betto, Federico Lolli
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

#include "Follower.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

using namespace miosix;

namespace Boardcore
{

Follower::Follower(float updatePeriod)
    : updatePeriod(updatePeriod), targetAngles({0, 0})
{
}

void Follower::setAntennaCoordinates(const Boardcore::GPSData& gpsData)
{
    antennaCoordinates = {gpsData.latitude, gpsData.longitude, gpsData.height};
    Boardcore::Logger::getInstance().log(
        static_cast<LogAntennasCoordinates>(gpsData));
    antennaCoordinatesSet = true;
}

void Follower::setInitialRocketCoordinates(const Boardcore::GPSData& gpsData)
{
    if (rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Rocket coordinates already set");
        return;
    }

    initialRocketCoordinates = {gpsData.latitude, gpsData.longitude,
                                gpsData.height};
    Boardcore::Logger::getInstance().log(
        static_cast<LogRocketCoordinates>(gpsData));
    rocketCoordinatesSet = true;
}

void Follower::setLastAntennaAttitude(const VN300Data& attitudeData)
{
    Lock<FastMutex> lock(lastAntennaAttitudeMutex);
    lastAntennaAttitude = attitudeData;
}

VN300Data Follower::getLastAntennaAttitude()
{
    Lock<FastMutex> lock(lastAntennaAttitudeMutex);
    return lastAntennaAttitude;
}

void Follower::setLastRocketNasState(const NASState nasState)
{
    Lock<FastMutex> lock(lastRocketNasStateMutex);
    lastRocketNasState      = nasState;
    firstAntennaAttitudeSet = true;
}

NASState Follower::getLastRocketNasState()
{
    Lock<FastMutex> lock(lastRocketNasStateMutex);
    return lastRocketNasState;
}

FollowerState Follower::getState()
{
    miosix::Lock<miosix::FastMutex> lock(stateMutex);
    return state;
}

void Follower::setState(const FollowerState& newState)
{
    miosix::Lock<miosix::FastMutex> lock(stateMutex);
    state = newState;
}

bool Follower::init()
{
    if (!antennaCoordinatesSet || !rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Antenna or rocket coordinates not set");
        return false;
    }

    // Antenna Coordinates
    Eigen::Vector2f antennaCoord{antennaCoordinates.head<2>()};
    // Rocket coordinates
    Eigen::Vector2f rocketCoord{initialRocketCoordinates.head<2>()};

    initialAntennaRocketDistance =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    LOG_INFO(logger, "Initial antenna - rocket distance: [{}, {}] [m]\n",
             initialAntennaRocketDistance[0], initialAntennaRocketDistance[1]);

    return true;
}

void Follower::step()
{
    NASState lastRocketNasState = getLastRocketNasState();

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition = {lastRocketNasState.n, lastRocketNasState.e,
                                lastRocketNasState.d};

    // Calculate the antenna target angles from the NED rocket coordinates
    targetAngles = rocketPositionToAntennaAngles(rocketPosition);

    // If attitude data has never been set, do not actuate the steppers
    if (!firstAntennaAttitudeSet)
    {
        LOG_ERR(logger, "Antenna attitude not set");
        return;
    }

    VN300Data vn300 = getLastAntennaAttitude();

    // Calculate the amount to move from the current position
    AntennaAngles diffAngles{targetAngles.yaw - vn300.yaw,
                             targetAngles.pitch - vn300.pitch};

    // Rotate in the shortest direction
    if (diffAngles.yaw > 180)
    {
        diffAngles.yaw -= 360;
    }
    else if (diffAngles.yaw < -180)
    {
        diffAngles.yaw += 360;
    }

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed =
        std::abs((diffAngles.yaw * 1000) / (360 * updatePeriod));
    float verticalSpeed =
        std::abs((diffAngles.pitch * 1000) / (360 * updatePeriod));

    // Update the state of the follower
    FollowerState state;
    state.timestamp       = TimestampTimer::getTimestamp();
    state.yaw             = diffAngles.yaw;
    state.pitch           = diffAngles.pitch;
    state.horizontalSpeed = horizontalSpeed;
    state.verticalSpeed   = verticalSpeed;
    setState(state);

#ifndef NDEBUG
    std::cout << "[FOLLOWER] STEPPER "
              << "Angles: [" << state.yaw << ", " << state.pitch << "] "
              << "Speed: [" << state.horizontalSpeed << ", "
              << state.verticalSpeed << "]   VN300 measure: [" << vn300.yaw
              << ", " << vn300.pitch << "]\n";
#endif
}

AntennaAngles Follower::rocketPositionToAntennaAngles(
    const NEDCoords& rocketNed)
{
    // NED coordinates of the rocket in the NED antenna frame
    NEDCoords ned = {rocketNed.n + initialAntennaRocketDistance.x(),
                     rocketNed.e + initialAntennaRocketDistance.y(),
                     rocketNed.d};

    AntennaAngles angles;
    // Calculate the horizontal angle relative to the NED frame
    // std::atan2 outputs angles in radians, convert to degrees
    angles.yaw = std::atan2(ned.e, ned.n) / EIGEN_PI * 180;

    float distance = std::sqrt(ned.n * ned.n + ned.e * ned.e);
    // Calculate the vertical angle relative to the NED frame
    angles.pitch = std::atan2(-ned.d, distance) / EIGEN_PI * 180;

    return angles;
}

}  // namespace Boardcore
