/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Pietro Bortolus
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

namespace Boardcore
{
struct ERegData
{
    uint64_t timestamp;
    float downstreamPressure;
    float upstreamPressure;
    float filteredDownstreamPressure;
    float filteredUpstreamPressure;
    float servoPosition;

    ERegData()
    {
        timestamp                  = 0;
        downstreamPressure         = 0.0f;
        upstreamPressure           = 0.0f;
        filteredDownstreamPressure = 0.0f;
        filteredUpstreamPressure   = 0.0f;
        servoPosition              = 0.0f;
    }

    ERegData(uint64_t timestamp, float downstreamPressure,
             float upstreamPressure, float filteredDownstreamPressure,
             float filteredUpstreamPressure, float servoPosition)
        : timestamp(timestamp), downstreamPressure(downstreamPressure),
          upstreamPressure(upstreamPressure),
          filteredDownstreamPressure(filteredDownstreamPressure),
          filteredUpstreamPressure(filteredUpstreamPressure),
          servoPosition(servoPosition)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ERegData,
                          FIELD_DEF(timestamp) FIELD_DEF(downstreamPressure)
                              FIELD_DEF(upstreamPressure)
                                  FIELD_DEF(filteredDownstreamPressure)
                                      FIELD_DEF(filteredUpstreamPressure)
                                          FIELD_DEF(servoPosition));
    }
};

}  // namespace Boardcore
