/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Author: Vincenzo Santomarco, Alberto Nidasio
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

#include <algorithms/NAS/NASState.h>
#include <math.h>
#include <utils/Constants.h>

#include <Eigen/Eigen>

namespace Boardcore
{

class TrajectoryPoint
{
public:
    float z;   ///< Vertical position [m].
    float vz;  ///< Vertical velocity [m/s].

    TrajectoryPoint() : TrajectoryPoint(0, 0) {}

    TrajectoryPoint(float z, float vz) : z(z), vz(vz) {}

    /**
     * @brief Returns the distance squared between the two points.
     */
    static float distanceSquared(TrajectoryPoint a, TrajectoryPoint b)
    {
        return powf(a.z - b.z, 2) + powf(a.vz - b.vz, 2);
    }

    /**
     * @brief Returns the distance in height between the two given points.
     */
    static float zDistance(TrajectoryPoint a, TrajectoryPoint b)
    {
        return abs(a.z - b.z);
    }

    /**
     * @brief Returns the distance in vertical speed between the two points.
     */
    static float vzDistance(TrajectoryPoint a, TrajectoryPoint b)
    {
        return abs(a.vz - b.vz);
    }
};

/**
 * @brief Trajectory point with timestamp and velocity module.
 */
class TimedTrajectoryPoint : public TrajectoryPoint
{
public:
    uint64_t timestamp;
    float vMod;

    TimedTrajectoryPoint() : TrajectoryPoint(), timestamp(0), vMod(0) {}

    explicit TimedTrajectoryPoint(NASState state)
        : TrajectoryPoint(-state.d, -state.vd), timestamp(state.timestamp),
          vMod(Eigen::Vector3f{state.vn, state.ve, state.vd}.norm())
    {
    }

    float getMac() { return vMod / (Constants::CO + Constants::ALPHA * z); }
};

}  // namespace Boardcore
