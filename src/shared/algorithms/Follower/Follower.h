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

#pragma once

#include <algorithms/Algorithm.h>
#include <algorithms/NAS/NASState.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <sensors/SensorData.h>
#include <sensors/Vectornav/VN300/VN300Data.h>

#include <atomic>
#include <chrono>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "FollowerData.h"

namespace Boardcore
{

class Follower : public Algorithm
{
public:
    /**
     * @brief Constructor of the follower class.
     *
     * @param updatePeriod The period of update of the follower algorithm [ms].
     */
    explicit Follower(std::chrono::milliseconds updatePeriod);

    /**
     * @brief Check that both the antenna and rocket coordinates have been set.
     * @return `true` if initial rocket-antenna distance has been set,
     * `false` otherwise.
     */
    bool init() override;

    /**
     * @brief Setter for the GPS coordinates of the antenna.
     */
    void setAntennaCoordinates(const GPSData& gpsData);

    /**
     * @brief Setter for the GPS coordinates of the rocket's NAS origin
     * reference.
     */
    void setRocketNASOrigin(const GPSData& gpsData);

    /**
     * @brief Setter for the NAS state of the rocket.
     */
    void setLastRocketNasState(const NASState& nasState);

    /**
     * @brief Setter for the attitude of the antenna.
     * @param attitudeData The VN300 data containing the attitude of the antenna
     * (yaw, pitch)
     */
    void setLastAntennaAttitude(const VN300Data& attitudeData);

    /**
     * @brief Synchronized getter for the State of the follower algorithm.
     * @returns The state of the follower algorithm.
     */
    FollowerState getState();

    /**
     * @brief Getter for the target antenna position computed by the algorithm.
     * @returns The target antenna positions.
     */
    AntennaAngles getTargetAngles() { return targetAngles; }

private:
    /**
     * @brief Use the last rocket NAS state and initial rocket-antenna distance
     * to compute the target antenna angles, retrievable with `getTargetAngles`.
     */
    void step() override;

    /**
     * @brief Synchronized getter that returns a copy of the last antenna
     * attitude of the antenna.
     */
    VN300Data getLastAntennaAttitude();

    /**
     * @brief Synchronized getter that returns a copy of the last NAS state
     * of the rocket
     */
    NASState getLastRocketNasState();

    /**
     * @brief Calculates the target angles from the given NED coordinates that
     * the antenna should point to.
     */
    AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned);

    /**
     * @brief Synchronized setter for the state of the follower algorithm.
     * @warning Should NOT be called if not in a test.
     */
    void setState(const FollowerState& newState);

    /**
     * @brief Get for the GPS coordinates of the antenna.
     */
    Eigen::Vector3f getAntennaCoordinates();

    /**
     * @brief Get for the GPS coordinates of the rocket's NAS origin reference.
     */
    Eigen::Vector3f getRocketNASOrigin();

    // actuation update period [s]
    float updatePeriod;

    // max number of retries for GPS data acquisition
    const uint8_t maxInitRetries = 120;

    bool antennaCoordinatesSet   = false;
    bool rocketCoordinatesSet    = false;
    bool firstAntennaAttitudeSet = false;

    VN300Data lastAntennaAttitude;
    miosix::FastMutex lastAntennaAttitudeMutex;

    NASState lastRocketNasState;
    miosix::FastMutex lastRocketNasStateMutex;

    // GPS coordinates of the antenna [lat, lon, alt] [deg, deg, m]
    Eigen::Vector3f antennaCoordinates;
    miosix::FastMutex antennaCoordinatesMutex;
    // GPS coordinates of the NAS origin taken from reference origin [lat, lon,
    // alt] [deg, deg, m]
    Eigen::Vector3f rocketNASOrigin;
    miosix::FastMutex rocketNASOriginMutex;
    // Initial distance between the antenna and the rocket while in ramp [lat,
    // lon, alt] [deg, deg, m]
    Eigen::Vector2f initialAntennaRocketDistance;

    // Target yaw and pitch of the system [deg, deg]
    AntennaAngles targetAngles;

    FollowerState state;
    miosix::FastMutex stateMutex;

    PrintLogger logger = Logging::getLogger("Follower");
};

}  // namespace Boardcore
