/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <sensors/SensorData.h>

#include <reflect.hpp>

namespace Boardcore
{

struct NEDCoords
{
    float n = 0;
    float e = 0;
    float d = 0;
};

/**
 * @brief A structure for storing angles relative to the NED frame.
 */
struct AntennaAngles
{
    uint64_t timestamp = 0;

    float yaw;    //!< Angle between the X axis (N axis) and the target position
                  //!< on the XY plane (NE plane), positive clockwise [deg]
    float pitch;  //!< Angle between the XY plane (NE plane) and the target
    //!< position, positive UP [deg]

    AntennaAngles() : timestamp{0}, yaw{0}, pitch{0} {};
    AntennaAngles(uint64_t timestamp, float yaw, float pitch)
        : timestamp{timestamp}, yaw(yaw), pitch(pitch) {};
    AntennaAngles(uint64_t timestamp, float yaw, float pitch,
                  uint32_t nrPropagations)
        : timestamp{timestamp}, yaw(yaw), pitch(pitch) {};

    constexpr static auto reflect()
    {
        return STRUCT_DEF(AntennaAngles,
                          FIELD_DEF(timestamp) FIELD_DEF(yaw) FIELD_DEF(pitch));
    }
};

/**
 * @brief A structure for storing angles relative to the NED frame and the
 * number of propagations that produce such angle, 0 if no propagation step has
 * been used. Used for logging.
 */
struct AntennaAnglesLog : public AntennaAngles
{
    uint32_t nrPropagations =
        0;  //!< Nr of propagations by the propagator (0 if no propagation)

    AntennaAnglesLog() : AntennaAngles(), nrPropagations(0) {};
    AntennaAnglesLog(uint64_t timestamp, float yaw, float pitch)
        : AntennaAngles(timestamp, yaw, pitch), nrPropagations{0} {};
    AntennaAnglesLog(uint64_t timestamp, float yaw, float pitch,
                     uint32_t nrPropagations)
        : AntennaAngles(timestamp, yaw, pitch),
          nrPropagations{nrPropagations} {};
    AntennaAnglesLog(AntennaAngles angle, uint32_t nrPropagations)
        : AntennaAngles(angle), nrPropagations{nrPropagations} {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AntennaAnglesLog,
                          EXTEND_DEF(AntennaAngles) FIELD_DEF(nrPropagations));
    }
};

/**
 * @brief State of the Follower algorithm, with the angles and speeds
 *
 */
struct FollowerState
{
    uint64_t timestamp;
    float yaw;    // [deg]
    float pitch;  // [deg]
    float horizontalSpeed;
    float verticalSpeed;

    FollowerState()
        : timestamp(0), yaw(0), pitch(0), horizontalSpeed(0), verticalSpeed(0)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(FollowerState,
                          FIELD_DEF(timestamp) FIELD_DEF(yaw) FIELD_DEF(pitch)
                              FIELD_DEF(horizontalSpeed)
                                  FIELD_DEF(verticalSpeed));
    }
};

/**
 * @brief A structure for logging the ARP system coordinates set in the
 * Follower.
 */
struct LogAntennasCoordinates : public GPSData
{
    LogAntennasCoordinates() = default;

    explicit LogAntennasCoordinates(const GPSData& data) : GPSData(data) {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LogAntennasCoordinates, EXTEND_DEF(GPSData));
    }
};

/**
 * @brief A structure for logging the Rocket coordinates set in the Follower.
 */
struct LogRocketCoordinates : public GPSData
{
    LogRocketCoordinates() = default;

    explicit LogRocketCoordinates(const GPSData& data) : GPSData(data) {}

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LogRocketCoordinates, EXTEND_DEF(GPSData));
    }
};
}  // namespace Boardcore
