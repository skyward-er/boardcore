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
    float yaw;    //!< Angle between the X axis (N axis) and the target position
                  //!< on the XY plane (NE plane), positive anti-clockwise [deg]
    float pitch;  //!< Angle between the XY plane (NE plane) and the target
                  //!< position [deg]
};

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

    static std::string header()
    {
        return "timestamp,yaw,pitch,horizontalSpeed,verticalSpeed\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << yaw << "," << pitch << "," << horizontalSpeed
           << "," << verticalSpeed << "\n";
    }
};

/**
 * @brief A structure for logging the ARP system coordinates set in the
 * Follower.
 */
struct LogAntennasCoordinates : public GPSData
{
    explicit LogAntennasCoordinates(const GPSData& data) : GPSData(data) {}

    LogAntennasCoordinates() {}
};

/**
 * @brief A structure for logging the Rocket coordinates set in the Follower.
 */
struct LogRocketCoordinates : public GPSData
{
    explicit LogRocketCoordinates(const GPSData& data) : GPSData(data) {}

    LogRocketCoordinates() {}
};
}  // namespace Boardcore
