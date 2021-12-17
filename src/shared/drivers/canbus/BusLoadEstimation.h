/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
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

#include "CanData.h"

using miosix::FastMutex;
using miosix::Lock;

namespace Boardcore
{

namespace Canbus
{

class BusLoadEstimation
{
    static constexpr uint16_t BUFFER_LEN = 100;

public:
    struct BusLoadInfo
    {
        float payload_bit_rate;
        float total_bit_rate;
        float load_percent;
    };

    BusLoadEstimation(uint32_t baud_rate) : baud_rate(baud_rate) {}

    void addPacket(CanPacket p)
    {
        Lock<FastMutex> l(mutex);
        c.put(PacketInfo{p.timestamp, p.length});
    }

    BusLoadInfo getLoadInfo()
    {
        Lock<FastMutex> l(mutex);
        if (c.count() < 2)
        {
            return {0, 0, 0};
        }

        float dt =
            (c.get(c.count() - 1).timestamp - c.get(0).timestamp) / 1000.0f;
        uint32_t size_payload = 0;
        uint32_t size_frames  = 0;

        for (size_t i = 0; i < c.count(); ++i)
        {
            size_payload += c.get(i).data_length * 8;
        }
        size_frames = size_payload + (64 + 8) * c.count();

        return BusLoadInfo{(size_payload / dt), (size_frames / dt),
                           ((size_frames / dt) / baud_rate) * 100};
    }

private:
    struct PacketInfo
    {
        uint32_t timestamp;
        uint8_t data_length;
    };

    CircularBuffer<PacketInfo, BUFFER_LEN> c;
    mutable FastMutex mutex;
    uint32_t baud_rate;
};

}  // namespace Canbus

}  // namespace Boardcore