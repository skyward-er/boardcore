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

#include <miosix.h>

#include <deque>
#include <limits>

using miosix::FastMutex;
using miosix::Lock;
using std::deque;

namespace Boardcore
{

struct DataRateResult
{
    float dataRate;
    float packetLoss;
    float packetsPerSecond;
};

/**
 * Helper class for calculating the throughput of communications. Just
 * pass the size of a packet to addPacket(...) as soon as the packet is
 * received.
 */
class ThroughputCalculator
{
public:
    /**
     * @brief Creates a new ThroughputCalculator
     *
     * @param expectedPktInterval Expected packet delivery interval for packet
     * loss estimation
     * @param windowDuration Duration of the observation window. Longer windows
     * means more stable values but slow response to abrupt changes
     */
    ThroughputCalculator(unsigned int expectedPktInterval,
                         unsigned int windowDuration = 2000)
        : expectedPktInterval(expectedPktInterval),
          windowDuration(windowDuration)
    {
    }

    /**
     * @brief Signal that a new packet has arrived. Call this as soon as the
     * packet is available
     *
     * @param packetSize The size of the packet
     */
    void addPacket(size_t packetSize)
    {
        Lock<FastMutex> lock(mutexPkt);

        long long ts = miosix::getTick();

        unsigned int interval = 0;
        if (packets.size() > 0)
        {
            interval = static_cast<unsigned int>(ts - packets.back().timestamp);
        }

        packets.push_back({ts, packetSize, interval});

        removeOldPackets(ts);
    }

    /**
     * @brief Returns the datarate in Bytes/second averaged over the duration of
     * the window
     */
    float getDataRate()
    {
        Lock<FastMutex> lock(mutexPkt);

        long long ts = miosix::getTick();
        removeOldPackets(ts);

        float sum = 0;
        for (size_t i = 0; i < packets.size(); i++)
        {
            sum += packets[i].size;
        }
        return sum / (windowDuration / 1000.0f);
    }

    /**
     * @brief Returns the packet loss percentage ([0-1]) based on the expected
     * packet delivery interval
     */
    float getPacketLoss()
    {
        Lock<FastMutex> lock(mutexPkt);
        long long ts = miosix::getTick();
        removeOldPackets(ts);

        float avgInterval = std::numeric_limits<float>::infinity();
        if (packets.size() > 0)
        {
            avgInterval = packets[0].interval;
            for (size_t i = 1; i < packets.size(); i++)
            {
                avgInterval += packets[i].interval;
            }

            avgInterval /= packets.size();
        }

        return 1 - expectedPktInterval / avgInterval;
    }

    /**
     * @brief Returns the pps value averaged over the duration of the window
     */
    float getPacketsPerSecond()
    {
        Lock<FastMutex> lock(mutexPkt);
        long long ts = miosix::getTick();
        removeOldPackets(ts);

        return (float)packets.size() / (windowDuration / 1000.0f);
    }

    DataRateResult getResult()
    {
        DataRateResult r;
        r.dataRate         = getDataRate();
        r.packetLoss       = getPacketLoss();
        r.packetsPerSecond = getPacketsPerSecond();

        return r;
    }

private:
    struct Packet
    {
        long long timestamp;
        size_t size;
        unsigned int interval;
    };

    void removeOldPackets(long long ts)
    {
        while (packets.size() > 0 &&
               packets.front().timestamp < ts - windowDuration)
        {
            packets.pop_front();
        }
    }

    unsigned int expectedPktInterval;
    unsigned int windowDuration;
    deque<Packet> packets;

    FastMutex mutexPkt;
};

}  // namespace Boardcore
