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

#include <math/Stats.h>

#include <array>
#include <ostream>
#include <string>

using std::array;
using std::string;
using std::to_string;

namespace Boardcore
{

struct XbeeConfig
{
    long long timestamp;
    bool tx_enabled            = false;
    uint16_t packet_size       = 256;
    unsigned int send_interval = 0;
    bool freq_hop              = true;
    bool data_rate_80k         = false;

    void print()
    {
        printf("+++XBee configuration+++\n\n");
        printf("Tx: %s, pkt: %u B, int: %d ms\n",
               tx_enabled ? "enabled" : "disabled", packet_size, send_interval);
        printf("Freq hop: %s, data rate: %s \n", freq_hop ? "on" : "off",
               data_rate_80k ? "80 kbps" : "10 kbps");
    }

    static string header()
    {
        return "timestamp,tx_enabled,packet_size,send_interval,freq_hop,data_"
               "rate_80k\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << tx_enabled << "," << packet_size << ","
           << send_interval << "," << freq_hop << "," << data_rate_80k << "\n";
    }
};

struct TxData
{
    long long timestamp               = 0LL;
    unsigned int packet_size          = 0;
    unsigned int time_since_last_send = 0;
    unsigned int time_to_send         = 0;
    unsigned int tx_success_counter   = 0;
    unsigned int tx_fail_counter      = 0;

    static string header()
    {
        return "timestamp,packet_size,time_since_last_send,time_to_send,tx_"
               "success_cnt,tx_fail_cnt\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << packet_size << "," << time_since_last_send
           << "," << time_to_send << "," << tx_success_counter << ","
           << tx_fail_counter << "\n";
    }
};

struct RxData
{
    long long timestamp;
    size_t pkt_size                 = 0;
    long long last_packet_timestamp = 0;
    int RSSI                        = 0;
    unsigned int rcv_count          = 0;
    unsigned int packets_lost       = 0;
    unsigned int rcv_errors         = 0;
    unsigned int rcv_wrong_payload  = 0;

    static string header()
    {
        return "timestamp,packet_size,last_packet_timestamp,RSSI,rcv_cnt,rcv_"
               "errors,"
               "rcv_wrong\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << pkt_size << "," << last_packet_timestamp
           << "," << RSSI << "," << rcv_count << "," << rcv_errors << ","
           << rcv_wrong_payload << "\n";
    }
};

struct EnergyScanData
{
    long long timestamp;
    int channel_data[30];

    EnergyScanData() = default;

    EnergyScanData(long long ts, array<int, 30> scan)
    {
        for (int i = 0; i < 30; i++)
        {
            channel_data[i] = scan[i];
        }

        timestamp = ts;
    }

    static string header()
    {
        string out = "timestamp";
        for (int i = 0; i < 30; i++)
        {
            out += ",channel_" + to_string(i);
        }
        return out + "\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp;

        for (int i = 0; i < 30; i++)
        {
            os << "," << channel_data[i];
        }

        os << "\n";
    }
};

}  // namespace Boardcore
