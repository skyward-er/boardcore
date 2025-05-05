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

static constexpr float YAW_GAIN_LIMIT =
    1.0;  ///< Max limit for the Yaw gain, cannot be set more
static constexpr float PITCH_GAIN_LIMIT =
    1.0;  ///< Max limit for the pirch gain cannot be set more

/**
 * @brief Follower class to output the yaw ad pitch necessary to track from the
 * GPS origin the rocket. Computes the angle to follow the rocket using its NAS
 * origin, NED position and velocity
 *
 */
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
     * @note No checks for the GPS fix are done
     */
    void setAntennaCoordinates(const GPSData& gpsData);

    /**
     * @brief Setter for the GPS coordinates of the rocket's NAS origin
     * reference.
     * @note No checks for the GPS fix are done
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
     * @brief Set the maximum gain for the yaw and pitch
     *
     * @param yawGainNew the gain for the yaw
     * @param pitchGainNew the gain for the pitch
     * @return true if set correctly
     * @return false if negative or over the maximum limits
     */
    bool setMaxGain(float yawGainNew, float pitchGainNew);

    /**
     * @brief Synchronized getter for the State of the follower algorithm.
     * @returns The state of the follower algorithm.
     */
    FollowerState getState();

    /**
     * @brief Getter for the target antenna position computed by the algorithm.
     * @returns The target antenna positions.
     */
    AntennaAngles getTargetAngles();

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
     *
     * @note Called by a mutex-protected function
     */
    AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned);

    /**
     * @brief Synchronized setter for the state of the follower algorithm.
     * @warning Should NOT be called if not in a test.
     */
    void setState(const FollowerState& newState);

    /**
     * @brief Get for the [lat, lon] coordinates of the antenna.
     */
    Eigen::Vector2f getAntennaCoordinates();

    /**
     * @brief Get for the GPS coordinates of the rocket's NAS origin reference.
     */
    Eigen::Vector3f getRocketNASOrigin();

    // actuation update period [ms]
    std::chrono::milliseconds updatePeriod;
    // Initialization flag
    std::atomic<bool> isInit{false};

    // max number of retries for GPS data acquisition
    const uint8_t maxInitRetries = 120;

    bool antennaCoordinatesSet = false;
    bool rocketCoordinatesSet  = false;
    bool lastRocketNasStateSet = false;
    std::atomic<bool> firstAntennaAttitudeSet{false};

    VN300Data lastAntennaAttitude;

    NASState lastRocketNasState;

    // TODO: See if assumption has sense...
    /* GPS coordinates of the antenna [lat, lon] [deg, deg],
     altitude is considered same as NAS Origin */
    Eigen::Vector2f antennaCoordinates;
    /* GPS coordinates of the NAS origin taken from reference origin [lat, lon,
     alt] [deg, deg, m] */
    Eigen::Vector3f rocketNASOrigin;
    /* Distance between the antenna and the rocket [lat,
     lon, alt] [deg, deg, m] */
    Eigen::Vector2f antennaRocketDistance;

    // Target yaw and pitch of the system [deg, deg]
    AntennaAngles targetAngles;

    FollowerState state;

    PrintLogger logger = Logging::getLogger("Follower");

    // General mutex for the follower
    miosix::FastMutex followerMutex;

    float yawGain   = YAW_GAIN_LIMIT;    ///< Gain on the yaw
    float pitchGain = PITCH_GAIN_LIMIT;  ///< Gain on the pitch
};

}  // namespace Boardcore
