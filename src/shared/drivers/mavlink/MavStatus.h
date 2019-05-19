/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#include <mavlink_skyward_lib/mavlink_lib/mavlink_types.h>
#include <ostream>
#include <string>

struct MavStatus
{
    uint64_t timestamp;
    uint16_t n_send_queue;  // current len of the occupied portion of the queue
    uint16_t max_send_queue;     // max occupied len of the queue
    uint16_t n_send_errors;      // Number of failed sends
    uint16_t n_dropped_packets;  // number of packet drops
    mavlink_status_t mav_stats;

    static std::string header()
    {
        return "timestamp,n_send_queue,max_send_queue,n_send_errors,n_dropped_"
               "packets,mav_stats."
               "buffer_overrun,mav_stats.msg_received,mav_stats.parse_error,"
               "mav_stats.parse_state, "
               "mav_stats.packet_idx,mav_stats.current_rx_seq,mav_stats."
               "current_tx_seq,mav_stats.packet_rx_success_count,mav_stats."
               "packet_rx_drop_count\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << n_send_queue << "," << max_send_queue << ","
           << n_send_errors << "," << n_dropped_packets << ","
           << (int)mav_stats.msg_received << ","
           << (int)mav_stats.buffer_overrun << "," << (int)mav_stats.parse_error
           << "," << (int)mav_stats.parse_state << ","
           << (int)mav_stats.packet_idx << "," << (int)mav_stats.current_rx_seq
           << "," << (int)mav_stats.current_tx_seq << ","
           << mav_stats.packet_rx_success_count << ","
           << mav_stats.packet_rx_drop_count << "\n";
    }
};