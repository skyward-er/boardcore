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
    bool txEnabled            = false;
    uint16_t packetSize       = 256;
    unsigned int sendInterval = 0;
    bool freqHop              = true;
    bool dataRate80k          = false;

    void print()
    {
        printf("+++XBee configuration+++\n\n");
        printf("Tx: %s, pkt: %u B, int: %u ms\n",
               txEnabled ? "enabled" : "disabled", packetSize, sendInterval);
        printf("Freq hop: %s, data rate: %s \n", freqHop ? "on" : "off",
               dataRate80k ? "80 kbps" : "10 kbps");
    }

    static string header()
    {
        return "timestamp,txEnabled,packetSize,send_interval,freq_hop,data_"
               "rate_80k\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << txEnabled << "," << packetSize << ","
           << sendInterval << "," << freqHop << "," << dataRate80k << "\n";
    }
};

struct TxData
{
    long long timestamp            = 0LL;
    unsigned int packetSize        = 0;
    unsigned int timeSinceLastSend = 0;
    unsigned int timeToSend        = 0;
    unsigned int txSuccessCounter  = 0;
    unsigned int txFailCounter     = 0;

    static string header()
    {
        return "timestamp,packetSize,time_since_last_send,time_to_send,tx_"
               "success_cnt,tx_fail_cnt\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << packetSize << "," << timeSinceLastSend << ","
           << timeToSend << "," << txSuccessCounter << "," << txFailCounter
           << "\n";
    }
};

struct RxData
{
    long long timestamp;
    size_t pktSize                = 0;
    long long lastPacketTimestamp = 0;
    int RSSI                      = 0;
    unsigned int rcvCount         = 0;
    unsigned int packetsLost      = 0;
    unsigned int rcvErrors        = 0;
    unsigned int rcvWrongPayload  = 0;

    static string header()
    {
        return "timestamp,packetSize,last_packet_timestamp,RSSI,rcv_cnt,rcv_"
               "errors,"
               "rcv_wrong\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << pktSize << "," << lastPacketTimestamp << ","
           << RSSI << "," << rcvCount << "," << rcvErrors << ","
           << rcvWrongPayload << "\n";
    }
};

struct EnergyScanData
{
    long long timestamp;
    int channelData[30];

    explicit EnergyScanData(long long ts, const array<int, 30> scan)
    {
        for (int i = 0; i < 30; i++)
        {
            channelData[i] = scan[i];
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
            os << "," << channelData[i];
        }

        os << "\n";
    }
};

}  // namespace Boardcore
