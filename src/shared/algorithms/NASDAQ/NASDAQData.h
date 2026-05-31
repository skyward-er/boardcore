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

#include <reflect.hpp>

#include "algorithms/NASDAQ/NASDAQ0_types.h"

namespace Boardcore
{

struct NASDAQState
{
    uint64_t timestamp = 0;

    // Position
    float x = 0;
    float y = 0;
    float z = 0;

    // Velocity
    float vx = 0;
    float vy = 0;
    float vz = 0;

    NASDAQState() : timestamp(0) {};

    NASDAQState(uint64_t timestamp, const float position[3],
                const float velocity[3])
        : timestamp(timestamp), x(position[0]), y(position[1]), z(position[2]),
          vx(velocity[0]), vy(velocity[1]), vz(velocity[2]) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(NASDAQState,
                          FIELD_DEF(timestamp) FIELD_DEF(x) FIELD_DEF(y)
                              FIELD_DEF(z) FIELD_DEF(vx) FIELD_DEF(vy)
                                  FIELD_DEF(vz));
    }
};

// Check per timestamp esterno/interno

struct NASDAQLogsWrapper
{
    uint64_t obswTimestamp;

    NASDAQ0_types_h_::NASDAQLogs NASDAQLog;

    NASDAQLogsWrapper() : obswTimestamp(0) {};

    NASDAQLogsWrapper(uint64_t timestamp,
                      const NASDAQ0_types_h_::NASDAQLogs logs)
        : obswTimestamp(timestamp), NASDAQLog(logs) {};

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            NASDAQLogsWrapper,
            FIELD_DEF2(NASDAQLog, Timestamp) FIELD_DEF2(NASDAQLog, Position)
                FIELD_DEF2(NASDAQLog, Velocity)
                    FIELD_DEF2(NASDAQLog, CovarianceMatrixDiagonal)
                        FIELD_DEF2(NASDAQLog, BaroActivation)
                            FIELD_DEF2(NASDAQLog, GPSActivation)
                                FIELD_DEF2(NASDAQLog, ADAActovation));
    }
};

}  // namespace Boardcore
