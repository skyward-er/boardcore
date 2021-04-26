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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
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

struct DataRateResult
{
    float data_rate;
    float packet_loss;
    float packets_per_second;
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
     * @param expected_pkt_interval Expected packet delivery interval for packet
     * loss estimation
     * @param window_duration Duration of the observation window. Longer windows
     * means more stable values but slow response to abrupt changes
     */
    ThroughputCalculator(unsigned int expected_pkt_interval,
                         unsigned int window_duration = 2000)
        : expected_pkt_interval(expected_pkt_interval),
          window_duration(window_duration)
    {
    }

    /**
     * @brief Signal that a new packet has arrived. Call this as soon as the
     * packet is available
     *
     * @param packet_size The size of the packet
     */
    void addPacket(size_t packet_size)
    {
        Lock<FastMutex> lock(mutex_pkt);

        long long ts = miosix::getTick();

        unsigned int interval = 0;
        if (packets.size() > 0)
        {
            interval = static_cast<unsigned int>(ts - packets.back().timestamp);
        }

        packets.push_back({ts, packet_size, interval});

        removeOldPackets(ts);
    }

    /**
     * @brief Returns the datarate in Bytes/second averaged over the duration of
     * the window
     */
    float getDataRate()
    {
        Lock<FastMutex> lock(mutex_pkt);

        long long ts = miosix::getTick();
        removeOldPackets(ts);

        float sum = 0;
        for (size_t i = 0; i < packets.size(); i++)
        {
            sum += packets[i].size;
        }
        return sum / (window_duration / 1000.0f);
    }

    /**
     * @brief Returns the packet loss percentage ([0-1]) based on the expected packet
     * delivery interval 
     */
    float getPacketLoss()
    {
        Lock<FastMutex> lock(mutex_pkt);
        long long ts = miosix::getTick();
        removeOldPackets(ts);

        float avg_interval = std::numeric_limits<float>::infinity();
        if (packets.size() > 0)
        {
            avg_interval = packets[0].interval;
            for (size_t i = 1; i < packets.size(); i++)
            {
                avg_interval += packets[i].interval;
            }

            avg_interval /= packets.size();
        }

        return 1 - expected_pkt_interval / avg_interval;
    }

    /**
     * @brief Returns the pps value averaged over the duration of the window
     */
    float getPacketsPerSecond()
    {
        Lock<FastMutex> lock(mutex_pkt);
        long long ts = miosix::getTick();
        removeOldPackets(ts);

        return (float)packets.size() / (window_duration / 1000.0f);
    }

    DataRateResult getResult()
    {
        DataRateResult r;
        r.data_rate          = getDataRate();
        r.packet_loss        = getPacketLoss();
        r.packets_per_second = getPacketsPerSecond();

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
               packets.front().timestamp < ts - window_duration)
        {
            packets.pop_front();
        }
    }

    unsigned int expected_pkt_interval;
    unsigned int window_duration;
    deque<Packet> packets;

    FastMutex mutex_pkt;
};