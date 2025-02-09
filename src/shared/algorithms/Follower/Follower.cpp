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

Follower::Follower(std::chrono::milliseconds updPeriod)
    : updatePeriod{updPeriod}, targetAngles({0, 0, 0})
{
}

void Follower::setAntennaCoordinates(const GPSData& gpsData)
{
    Lock<FastMutex> lock(followerMutex);
    antennaCoordinates = {gpsData.latitude, gpsData.longitude};
    Boardcore::Logger::getInstance().log(LogAntennasCoordinates(gpsData));
    antennaCoordinatesSet = true;
}

void Follower::setRocketNASOrigin(const GPSData& gpsData)
{
    Lock<FastMutex> lock(followerMutex);
    rocketNASOrigin = {gpsData.latitude, gpsData.longitude, gpsData.height};
    Boardcore::Logger::getInstance().log(LogRocketCoordinates(gpsData));
    rocketCoordinatesSet = true;
}

void Follower::setLastAntennaAttitude(const VN300Data& attitudeData)
{
    Lock<FastMutex> lock(followerMutex);
    firstAntennaAttitudeSet = true;
    lastAntennaAttitude     = attitudeData;
}

VN300Data Follower::getLastAntennaAttitude()
{
    Lock<FastMutex> lock(followerMutex);
    return lastAntennaAttitude;
}

void Follower::setLastRocketNasState(const NASState& nasState)
{
    Lock<FastMutex> lock(followerMutex);
    lastRocketNasState    = nasState;
    lastRocketNasStateSet = true;
}

NASState Follower::getLastRocketNasState()
{
    Lock<FastMutex> lock(followerMutex);
    return lastRocketNasState;
}

FollowerState Follower::getState()
{
    miosix::Lock<miosix::FastMutex> lock(followerMutex);
    return state;
}

void Follower::setState(const FollowerState& newState)
{
    miosix::Lock<miosix::FastMutex> lock(followerMutex);
    state = newState;
}

AntennaAngles Follower::getTargetAngles()
{
    miosix::Lock<miosix::FastMutex> lock(followerMutex);
    return targetAngles;
}

bool Follower::setMaxGain(float yawGainNew, float pitchGainNew)
{
    // In case of negative or over the limit values, do not set the gains
    if (yawGainNew < 0 || yawGainNew > YAW_GAIN_LIMIT || pitchGainNew < 0 ||
        pitchGainNew > PITCH_GAIN_LIMIT)
        return false;

    yawGain   = yawGainNew;
    pitchGain = pitchGainNew;
    return true;
}

bool Follower::init()
{
    Lock<FastMutex> lock(followerMutex);
    if (!antennaCoordinatesSet || !rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Antenna or rocket coordinates not set");
        return false;
    }
    return true;
}

void Follower::step()
{
    AntennaAngles diffAngles;
    VN300Data vn300;
    NASState lastRocketNas;
    NEDCoords rocketPosition;

    // Read the data for the step computation
    {
        Lock<FastMutex> lock(followerMutex);

        if (!firstAntennaAttitudeSet)
        {
            LOG_ERR(logger, "Antenna attitude not set\n");
            return;
        }
        // TODO: See if needed to check the NAS or rather point to the NAS
        // origin if missing

        lastRocketNas = lastRocketNasState;
        vn300         = lastAntennaAttitude;

        // Local variable checks and updates
        // Getting the position of the rocket wrt the antennas in NED frame
        rocketPosition = {lastRocketNas.n, lastRocketNas.e, lastRocketNas.d};

        // Calculate the antenna target angles from the NED rocket coordinates
        targetAngles = rocketPositionToAntennaAngles(rocketPosition);

        // Calculate the amount to move from the current position
        diffAngles = {targetAngles.timestamp, targetAngles.yaw - vn300.yaw,
                      targetAngles.pitch - vn300.pitch};
    }

    // Rotate in the shortest direction
    diffAngles.yaw =
        std::min(yawGain, YAW_GAIN_LIMIT) * minimizeRotation(diffAngles.yaw);
    diffAngles.pitch = std::min(pitchGain, PITCH_GAIN_LIMIT) *
                       minimizeRotation(diffAngles.pitch);

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed =
        std::abs((diffAngles.yaw) /
                 (360 * (static_cast<float>(updatePeriod.count()) / 1000)));
    TRACE("[Follower] horizontalSpeed is: %f\n", horizontalSpeed);
    float verticalSpeed =
        std::abs((diffAngles.pitch) /
                 (360 * (static_cast<float>(updatePeriod.count()) / 1000)));
    TRACE("[Follower] Vertical speed is: %f\n", horizontalSpeed);

    // Update the state of the follower
    FollowerState newState;
    newState.timestamp       = TimestampTimer::getTimestamp();
    newState.yaw             = diffAngles.yaw;
    newState.pitch           = diffAngles.pitch;
    newState.horizontalSpeed = horizontalSpeed;
    newState.verticalSpeed   = verticalSpeed;

    // Write the new state for the follower
    {
        Lock<FastMutex> lockWrite(followerMutex);
        state = newState;
    }

#ifndef NDEBUG
    std::cout << "[FOLLOWER] STEPPER " << "Angles: [" << newState.yaw << ", "
              << newState.pitch << "] " << "Speed: ["
              << newState.horizontalSpeed << ", " << newState.verticalSpeed
              << "]   VN300 measure: [" << vn300.yaw << ", " << vn300.pitch
              << "]\n";
#endif
}

Eigen::Vector2f Follower::getAntennaCoordinates()
{
    Lock<FastMutex> lock(followerMutex);
    return Eigen::Vector2f{antennaCoordinates};
}

Eigen::Vector3f Follower::getRocketNASOrigin()
{
    Lock<FastMutex> lock(followerMutex);
    return Eigen::Vector3f{rocketNASOrigin};
}

AntennaAngles Follower::rocketPositionToAntennaAngles(
    const NEDCoords& rocketNed)
{
    // Antenna Coordinates
    Eigen::Vector2f antennaCoord = antennaCoordinates;

    // Rocket coordinates, w/out altitude
    Eigen::Vector2f rocketCoord = rocketNASOrigin.head<2>();

    antennaRocketDistance = Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    // NED coordinates of the rocket in the NED antenna frame
    NEDCoords ned = {rocketNed.n + antennaRocketDistance.x(),
                     rocketNed.e + antennaRocketDistance.y(), rocketNed.d};

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
