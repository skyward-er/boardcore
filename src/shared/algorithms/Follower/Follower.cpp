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

/**
 * @brief Minimize rotation angle.
 *
 * @param angle Angle of movement [deg]
 * @return The minimized rotation angle [deg]
 */
float minimizeRotation(float angle)
{
    if (angle > 180)
        angle -= 360;
    else if (angle < -180)
        angle += 360;

    return angle;
}

Follower::Follower(std::chrono::milliseconds updatePeriod)
    : updatePeriod(static_cast<float>(updatePeriod.count()) / 1000),
      targetAngles({0, 0, 0}), firstAntennaAttitudeSet(false), isInit(false)
{
}

void Follower::setAntennaCoordinates(const Boardcore::GPSData& gpsData)
{
    Lock<FastMutex> lock(lastAntennaAttitudeMutex);
    antennaCoordinates = {gpsData.latitude, gpsData.longitude, gpsData.height};
    Boardcore::Logger::getInstance().log(LogAntennasCoordinates(gpsData));
    antennaCoordinatesSet = true;
}

void Follower::setRocketNASOrigin(const Boardcore::GPSData& gpsData)
{
    Lock<FastMutex> lock(rocketNASOriginMutex);
    rocketNASOrigin = {gpsData.latitude, gpsData.longitude, gpsData.height};
    Boardcore::Logger::getInstance().log(LogRocketCoordinates(gpsData));
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

void Follower::setLastRocketNasState(const NASState& nasState)
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

AntennaAngles Follower::getTargetAngles()
{
    miosix::Lock<miosix::FastMutex> lock(targetAnglesMutex);
    return targetAngles;
}

bool Follower::init()
{
    if (isInit)
        return true;
    if (!antennaCoordinatesSet || !rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Antenna or rocket coordinates not set");
        return false;
    }

    // Antenna Coordinates (w/out altitude)
    Eigen::Vector2f antennaCoord{getAntennaCoordinates().head<2>()};
    // Rocket coordinates (w/out altitude)
    Eigen::Vector2f rocketCoord{getRocketNASOrigin().head<2>()};

    initialAntennaRocketDistance =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    LOG_INFO(logger, "Initial antenna - rocket distance: [{}, {}] [m]\n",
             initialAntennaRocketDistance[0], initialAntennaRocketDistance[1]);

    isInit = true;
    return true;
}

void Follower::step()
{
    if (!isInit)
    {
        LOG_ERR(logger, "Not initialized Follower\n");
        return;
    }

    NASState lastRocketNas = getLastRocketNasState();

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition = {lastRocketNas.n, lastRocketNas.e,
                                lastRocketNas.d};

    AntennaAngles diffAngles;
    VN300Data vn300;

    {
        miosix::Lock<miosix::FastMutex> lockAngle(targetAnglesMutex);

        // Calculate the antenna target angles from the NED rocket coordinates
        targetAngles = rocketPositionToAntennaAngles(rocketPosition);

        {
            Lock<FastMutex> lockLastRocketNasState(lastRocketNasStateMutex);
            // If attitude data has never been set, do not actuate the steppers
            if (!firstAntennaAttitudeSet)
            {
                LOG_ERR(logger, "Antenna attitude not set\n");
                return;
            }
        }

        vn300 = getLastAntennaAttitude();

        // Calculate the amount to move from the current position
        diffAngles = {targetAngles.timestamp, targetAngles.yaw - vn300.yaw,
                      targetAngles.pitch - vn300.pitch};
    }

    // Rotate in the shortest direction
    diffAngles.yaw   = YAW_GAIN * minimizeRotation(diffAngles.yaw);
    diffAngles.pitch = PITCH_GAIN * minimizeRotation(diffAngles.pitch);

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed =
        std::abs((diffAngles.yaw * 1000) / (360 * updatePeriod));
    float verticalSpeed =
        std::abs((diffAngles.pitch * 1000) / (360 * updatePeriod));

    // Update the state of the follower
    FollowerState newState;
    newState.timestamp       = TimestampTimer::getTimestamp();
    newState.yaw             = diffAngles.yaw;
    newState.pitch           = diffAngles.pitch;
    newState.horizontalSpeed = horizontalSpeed;
    newState.verticalSpeed   = verticalSpeed;
    setState(newState);

#ifndef NDEBUG
    std::cout << "[FOLLOWER] STEPPER " << "Angles: [" << newState.yaw << ", "
              << newState.pitch << "] " << "Speed: ["
              << newState.horizontalSpeed << ", " << newState.verticalSpeed
              << "]   VN300 measure: [" << vn300.yaw << ", " << vn300.pitch
              << "]\n";
#endif
}

Eigen::Vector3f Follower::getAntennaCoordinates()
{
    Lock<FastMutex> lock(antennaCoordinatesMutex);
    return Eigen::Vector3f{antennaCoordinates};
}

Eigen::Vector3f Follower::getRocketNASOrigin()
{
    Lock<FastMutex> lock(rocketNASOriginMutex);
    return Eigen::Vector3f{rocketNASOrigin};
}

AntennaAngles Follower::rocketPositionToAntennaAngles(
    const NEDCoords& rocketNed)
{
    // Antenna Coordinates
    Eigen::Vector2f antennaCoord = getAntennaCoordinates().head<2>();

    // Rocket coordinates
    Eigen::Vector2f rocketCoord = getRocketNASOrigin().head<2>();

    initialAntennaRocketDistance =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    // NED coordinates of the rocket in the NED antenna frame
    NEDCoords ned = {rocketNed.n + initialAntennaRocketDistance.x(),
                     rocketNed.e + initialAntennaRocketDistance.y(),
                     rocketNed.d};

    AntennaAngles angles;
    angles.timestamp = TimestampTimer::getTimestamp();

    // Calculate the horizontal angle relative to the NED frame
    // std::atan2 outputs angles in radians, convert to degrees
    angles.yaw = std::atan2(ned.e, ned.n) / EIGEN_PI * 180;

    float distance = std::sqrt(ned.n * ned.n + ned.e * ned.e);
    // Calculate the vertical angle relative to the NED frame
    angles.pitch = std::atan2(-ned.d, distance) / EIGEN_PI * 180;

    return angles;
}

}  // namespace Boardcore
