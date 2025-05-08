/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <array>
#include <iostream>
#include <ostream>
#include <reflect.hpp>
#include <string>

using std::array;
using std::string;
using std::to_string;

namespace Boardcore
{

struct XbeeConfig
{
    int64_t timestamp;
    bool txEnabled        = false;
    uint16_t packetSize   = 256;
    uint32_t sendInterval = 0;
    bool freqHop          = true;
    bool dataRate80k      = false;

    void print()
    {
        std::cout << "+++XBee configuration+++\n\n";
        std::cout << "Tx: " << (txEnabled ? "enabled" : "disabled")
                  << ", pkt: " << packetSize << " B, int: " << sendInterval
                  << " ms\n";
        std::cout << "Freq hop: " << (freqHop ? "on" : "off")
                  << ", data rate: " << (dataRate80k ? "80 kbps" : "10 kbps")
                  << "\n";
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(XbeeConfig,
                          FIELD_DEF(timestamp) FIELD_DEF(txEnabled)
                              FIELD_DEF(packetSize) FIELD_DEF(sendInterval)
                                  FIELD_DEF(freqHop) FIELD_DEF(dataRate80k));
    }
};

struct TxData
{
    int64_t timestamp          = 0LL;
    uint32_t packetSize        = 0;
    uint32_t timeSinceLastSend = 0;
    uint32_t timeToSend        = 0;
    uint32_t txSuccessCounter  = 0;
    uint32_t txFailCounter     = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TxData,
                          FIELD_DEF(timestamp) FIELD_DEF(packetSize)
                              FIELD_DEF(timeSinceLastSend) FIELD_DEF(timeToSend)
                                  FIELD_DEF(txSuccessCounter)
                                      FIELD_DEF(txFailCounter));
    }
};

struct RxData
{
    int64_t timestamp;
    size_t pktSize              = 0;
    int64_t lastPacketTimestamp = 0;
    int32_t RSSI                = 0;
    uint32_t rcvCount           = 0;
    uint32_t packetsLost        = 0;
    uint32_t rcvErrors          = 0;
    uint32_t rcvWrongPayload    = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RxData,
                          FIELD_DEF(timestamp) FIELD_DEF(pktSize)
                              FIELD_DEF(lastPacketTimestamp) FIELD_DEF(RSSI)
                                  FIELD_DEF(rcvCount) FIELD_DEF(packetsLost)
                                      FIELD_DEF(rcvErrors)
                                          FIELD_DEF(rcvWrongPayload));
    }
};

struct EnergyScanData
{
    int64_t timestamp;
    int32_t channelData[30];

    EnergyScanData() = default;

    EnergyScanData(int64_t ts, const array<int32_t, 30> scan)
    {
        for (int i = 0; i < 30; i++)
            channelData[i] = scan[i];

        timestamp = ts;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(EnergyScanData,
                          FIELD_DEF(timestamp) FIELD_DEF(channelData));
    }
};

}  // namespace Boardcore
