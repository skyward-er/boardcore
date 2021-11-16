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

#include <cstdint>
#include <cstdio>
#include <ostream>
#include <string>

#include "math/Stats.h"

namespace Boardcore
{

namespace Xbee
{

struct XbeeStatus
{
    long long timestamp = 0LL;

    uint8_t last_tx_status_error = 0;
    uint8_t last_tx_status       = 0;

    StatsResult time_to_send_stats;

    unsigned int tx_timeout_count = 0;

    unsigned int rx_dropped_buffers = 0;

    unsigned int frame_buf_max_length = 0;

    static std::string header()
    {
        return "timestamp,last_tx_status_error,last_tx_status,tts_stats.min,"
               "tts_stats.max,tts_stats.mean,tts_stats.stddev,tts_stats.n_"
               "samples,tx_timeout_count,rx_dropped_buffers,frame_buf_max_"
               "length\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)last_tx_status_error << ","
           << (int)last_tx_status << "," << time_to_send_stats.minValue << ","
           << time_to_send_stats.maxValue << "," << time_to_send_stats.mean
           << "," << time_to_send_stats.stdev << ","
           << time_to_send_stats.nSamples << "," << tx_timeout_count << ","
           << rx_dropped_buffers << "," << frame_buf_max_length << "\n";
    }
};

}  // namespace Xbee

}  // namespace Boardcore
