/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <mavlink_lib/mavlink_types.h>

#include <ostream>
#include <string>

namespace Boardcore
{

struct MavlinkStatus
{
    uint64_t timestamp;
    uint16_t nSendQueue;  ///< Current len of the occupied portion of the queue
    uint16_t maxSendQueue;     ///< Max occupied len of the queue
    uint16_t nSendErrors;      ///< Number of failed sends
    uint16_t nDroppedPackets;  ///< Number of packet drops
    mavlink_status_t mavStats;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MavlinkStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(nSendQueue)
                              FIELD_DEF(maxSendQueue) FIELD_DEF(nSendErrors)
                                  FIELD_DEF(nDroppedPackets)
                                      FIELD_DEF(mavStats));
    }
};

}  // namespace Boardcore
