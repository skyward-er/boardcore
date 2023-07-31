/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <miosix.h>
#include <utils/collections/CircularBuffer.h>

#include <cstdint>

#include "CanDriverData.h"

using miosix::FastMutex;
using miosix::Lock;

namespace Boardcore
{

namespace Canbus
{

struct BusLoadInfo
{
    float payloadBitRate;
    float totalBitRate;
    float loadPercent;
};
class BusLoadEstimation
{
    static constexpr uint16_t BUFFER_LEN = 50;

public:
    BusLoadEstimation(uint32_t baudRate) : baudRate(baudRate) {}

    void addPacket(CanPacket p)
    {
        Lock<FastMutex> l(mutex);
        c.put(PacketInfo{p.timestamp, p.length});
    }

    BusLoadInfo getLoadInfo()
    {
        // implements a simple "moving avg"
        Lock<FastMutex> l(mutex);
        if (c.count() < 2)
        {
            return {0, 0, 0};
        }

        float dt =
            (c.get(c.count() - 1).timestamp - c.get(0).timestamp) / 1000.0f;
        uint32_t sizePayload = 0;
        uint32_t sizeFrames  = 0;

        for (size_t i = 0; i < c.count(); ++i)
        {
            sizePayload += c.get(i).dataLength * 8;
            sizeFrames += ((54 + c.get(i).dataLength * 8 - 1) / 4);
        }
        sizeFrames += sizePayload + 64 * c.count();

        return BusLoadInfo{(sizePayload / dt), (sizeFrames / dt),
                           ((sizeFrames / dt) / baudRate) * 100};
    }

private:
    struct PacketInfo
    {
        uint32_t timestamp;
        uint8_t dataLength;
    };

    CircularBuffer<PacketInfo, BUFFER_LEN> c;
    FastMutex mutex;
    uint32_t baudRate;
};

}  // namespace Canbus

}  // namespace Boardcore
