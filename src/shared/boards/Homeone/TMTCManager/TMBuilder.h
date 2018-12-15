/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Common.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>

namespace HomeoneBoard
{
namespace TMBuilder
{

/**
 * Parses a corresponding packed telemetry.
 */
static mavlink_message_t buildTelemetry(uint8_t requestedTelemetry) 
{
    mavlink_message_t responseMsg;

    // TODO: tm builder
    // mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg,
    //                                 request.msgid, request.seq);

    switch(requestedTelemetry)
    {
        case MavTMList::MAV_HOMEONE_TM_ID:
        break;

        case MavTMList::MAV_IGNITION_TM_ID:
        break;

        case MavTMList::MAV_NOSECONE_TM_ID:
        break;

        case MavTMList::MAV_HR_TM_ID:
        break;

        case MavTMList::MAV_LR_TM_ID:
        break;

        default:
        break;
    }

    return responseMsg;
}

} /* namespace TMBuilder */
} /* namespace HomeoneBoard */
