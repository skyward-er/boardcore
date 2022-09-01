/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <sensors/analog/Pitot/PitotData.h>

#include <cstring>

#include "CanProtocolData.h"

namespace Boardcore
{

inline Canbus::CanMessage toCanMessage(const PitotData& data)
{
    Canbus::CanMessage message;

    uint32_t deltaP, airspeed;
    memcpy(&deltaP, &(data.deltaP), sizeof(deltaP));
    memcpy(&airspeed, &(data.airspeed), sizeof(airspeed));

    message.id         = -1;
    message.length     = 2;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= deltaP;
    message.payload[1] = airspeed;

    return message;
}

inline PitotData pitotDataFromCanMessage(const Canbus::CanMessage& msg)
{
    PitotData data;

    uint32_t deltaP   = msg.payload[0];
    uint32_t airspeed = msg.payload[1];
    memcpy(&(data.deltaP), &deltaP, sizeof(data.deltaP));
    memcpy(&(data.airspeed), &airspeed, sizeof(data.airspeed));

    data.timestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

}  // namespace Boardcore
