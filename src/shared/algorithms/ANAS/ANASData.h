/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Lamon
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

#include <algorithms/ANAS/ANAS0_types.h>

#include <reflect.hpp>

namespace Boardcore
{

struct ANASState
{
    uint64_t timestamp = 0;

    // Position [m]
    float n = 0;  ///< North (x)
    float e = 0;  ///< East  (y)
    float d = 0;  ///< Down  (z)

    // Velocity [m/s]
    float vn = 0;  ///< Velocity North (x)
    float ve = 0;  ///< Velocity East  (y)
    float vd = 0;  ///< Velocity Down  (z)

    // Attitude as quaternion, from body to NED frame
    float qx = 0;  ///< Quaternion x
    float qy = 0;  ///< Quaternion y
    float qz = 0;  ///< Quaternion z
    float qw = 1;  ///< Quaternion w

    ANASState() : timestamp(0) {};

    ANASState(uint64_t timestamp, const float position[3],
                const float velocity[3], const float quaternions[4])
        : timestamp(timestamp), n(position[0]), e(position[1]), d(position[2]),
          vn(velocity[0]), ve(velocity[1]), vd(velocity[2]), qx(quaternions[0]),
          qy(quaternions[1]), qz(quaternions[2]), qw(quaternions[3]) {};

    // Ci sono due timestamp, uno interno (dell'alg) e uno esterno
    // (timestamptimer)

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ANASState,
                          FIELD_DEF(timestamp) FIELD_DEF(n) FIELD_DEF(e)
                              FIELD_DEF(d) FIELD_DEF(vn) FIELD_DEF(ve)
                                  FIELD_DEF(vd) FIELD_DEF(qx) FIELD_DEF(qy)
                                      FIELD_DEF(qz) FIELD_DEF(qw));
    }
};

struct ANASLogsData
{
    uint64_t timestamp;
    ANAS0_types_h_::NASLogs ANASLogs;

    ANASLogsData() : timestamp(0), ANASLogs() {};

    ANASLogsData(uint64_t timestamp, ANAS0_types_h_::NASLogs anasLogs)
        : timestamp(timestamp), ANASLogs(anasLogs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            ANASLogsData,
            FIELD_DEF(timestamp) FIELD_DEF2(ANASLogs, Velocity) FIELD_DEF2(
                ANASLogs, Quaternion) FIELD_DEF2(ANASLogs, Covariance)
                FIELD_DEF2(ANASLogs, BaroActivation) FIELD_DEF2(
                    ANASLogs, GPSActivation) FIELD_DEF2(ANASLogs, MagActivation)
                    FIELD_DEF2(ANASLogs, AccActivation)
                        FIELD_DEF2(ANASLogs, Position));
    }
};

// Non so se logghiamo la LinearCovariance

}  // namespace Boardcore
