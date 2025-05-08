/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <utils/Stats/Stats.h>

#include <cstdint>
#include <cstdio>
#include <ostream>
#include <reflect.hpp>
#include <string>

namespace Boardcore
{

namespace Xbee
{

struct XbeeStatus
{
    long long timestamp = 0LL;

    uint8_t lastTxStatusError = 0;
    uint8_t lastTxStatus      = 0;

    StatsResult timeToSendStats;

    unsigned int txTimeoutCount = 0;

    unsigned int rxDroppedBuffers = 0;

    unsigned int frameBufMaxLength = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            XbeeStatus,
            FIELD_DEF(timestamp) FIELD_DEF(lastTxStatusError) FIELD_DEF(
                lastTxStatus) FIELD_DEF2(timeToSendStats, minValue)
                FIELD_DEF2(timeToSendStats, maxValue) FIELD_DEF2(
                    timeToSendStats, mean) FIELD_DEF2(timeToSendStats, stdDev)
                    FIELD_DEF2(timeToSendStats, nSamples)
                        FIELD_DEF(txTimeoutCount) FIELD_DEF(rxDroppedBuffers)
                            FIELD_DEF(frameBufMaxLength));
    }
};

}  // namespace Xbee

}  // namespace Boardcore
