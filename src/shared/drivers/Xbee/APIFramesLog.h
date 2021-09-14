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

#include <algorithm>
#include <cstdint>
#include <ostream>
#include <string>
#include <utility>

#include "APIFrames.h"

using std::min;

/**
 * Classes used if you want to log an APIFrame, but do not want to log the base
 * APIFrame class, which is quite heavy on memory
 */

namespace Xbee
{

struct APIFrameLog
{
    long long timestamp;
    uint8_t frame_type;
    uint16_t frame_data_length;
    uint8_t frame_data[FRAME_DATA_SIZE];

    static bool fromAPIFrame(APIFrame& api, APIFrameLog* dest)
    {
        dest->timestamp         = api.timestamp;
        dest->frame_type        = api.frame_type;
        dest->frame_data_length = api.getFrameDataLength();

        memcpy(dest->frame_data, api.frame_data, dest->frame_data_length);

        return true;
    }

    static string header() { return "timestamp,length,frame_type\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << frame_data_length << "," << frame_type
           << "\n";
    }
};

struct ATCommandFrameLog
{
    long long timestamp;
    uint8_t frame_id = 0;
    char at_command[2];

    uint8_t command_data[MAX_AT_COMMAND_PARAMS_LENGTH];
    uint16_t command_data_length = 0;

    static bool toFrameType(APIFrame& api, ATCommandFrameLog* dest)
    {
        if (api.frame_type != FTYPE_AT_COMMAND)
        {
            return false;
        }

        if (api.getFrameDataLength() < MIN_AT_COMMAND_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_AT_COMMAND_FRAME_SIZE + MAX_AT_COMMAND_PARAMS_LENGTH)
        {
            return false;
        }
        ATCommandFrame* at = api.toFrameType<ATCommandFrame>();

        dest->timestamp = at->timestamp;
        dest->frame_id  = at->getFrameID();

        memcpy(dest->at_command, at->getATCommand(), 2);

        dest->command_data_length = at->getCommandDataLength();
        memcpy(dest->command_data, at->getCommandDataPointer(),
               dest->command_data_length);

        return true;
    }

    static string header() { return "timestamp,id,cmd,param_size,params\n"; }

    void print(std::ostream& os) const
    {
        char cmd[3];

        strncpy(cmd, at_command, 2);
        cmd[2] = '\0';

        os << timestamp << "," << (int)frame_id << "," << cmd << ","
           << command_data_length << ",";

        if (command_data_length > 0)
        {
            for (uint16_t i = 0; i < command_data_length; i++)
            {
                os << (int)command_data[i] << " ";
            }
        }
        else
        {
            os << "-";
        }

        os << "\n";
    }
};

struct TXRequestFrameLog
{
    long long timestamp = 0;
    uint8_t frame_id    = 0;

    uint64_t dest_address = 0;

    uint8_t broadcast_radius = 0;
    uint8_t transmit_options = 0;

    uint8_t rf_data[MAX_PACKET_PAYLOAD_LENGTH];
    uint16_t rf_data_length = 0;

    static bool toFrameType(APIFrame& api, TXRequestFrameLog* dest)
    {
        if (api.frame_type != FTYPE_TX_REQUEST)
        {
            return false;
        }

        if (api.getFrameDataLength() < MIN_TX_REQUEST_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_TX_REQUEST_FRAME_SIZE + MAX_PACKET_PAYLOAD_LENGTH)
        {
            return false;
        }

        TXRequestFrame* tx = api.toFrameType<TXRequestFrame>();

        dest->timestamp = api.timestamp;
        dest->frame_id  = tx->getFrameID();

        dest->dest_address = tx->getDestAddress();

        dest->broadcast_radius = tx->getBroadcastRadius();
        dest->transmit_options = tx->getTrasmitOptions();

        dest->rf_data_length = tx->getRFDataLength();
        memcpy(dest->rf_data, tx->getRFDataPointer(), dest->rf_data_length);

        return true;
    }

    static string header()
    {
        return "timestamp,id,dest_addr,broadcast_radius,tx_options,rf_data_"
               "len,first_uint32\n";
    }

    void print(std::ostream& os) const
    {
        uint32_t d;
        memcpy(&d, rf_data, sizeof(d));
        os << timestamp << "," << (int)frame_id << "," << dest_address << ","
           << (int)broadcast_radius << "," << (int)transmit_options << ","
           << rf_data_length << "," << d << "\n";
    }
};

struct ATCommandResponseFrameLog
{
    long long timestamp = 0;
    uint8_t frame_id    = 0;
    char at_command[2];
    uint8_t command_status = 0;

    uint8_t command_data[MAX_AT_COMMAND_RESPONSE_LENGTH];
    uint16_t command_data_length = 0;

    static bool toFrameType(APIFrame& api, ATCommandResponseFrameLog* dest)
    {
        if (api.frame_type != FTYPE_AT_COMMAND_RESPONSE)
        {
            return false;
        }

        if (api.getFrameDataLength() < MIN_AT_COMMAND_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_AT_COMMAND_FRAME_SIZE + MAX_AT_COMMAND_RESPONSE_LENGTH)
        {
            return false;
        }

        ATCommandResponseFrame* at = api.toFrameType<ATCommandResponseFrame>();

        dest->timestamp = api.timestamp;
        dest->frame_id  = at->getFrameID();
        memcpy(dest->at_command, at->getATCommand(), 2);

        dest->command_status = at->getCommandStatus();

        dest->command_data_length = at->getCommandDataLength();
        memcpy(dest->command_data, at->getCommandDataPointer(),
               dest->command_data_length);

        return true;
    }

    static string header()
    {
        return "timestamp,id,cmd,status,param_size,params\n";
    }

    void print(std::ostream& os) const
    {
        char cmd[3];

        strncpy(cmd, at_command, 2);
        cmd[2] = '\0';

        os << timestamp << "," << (int)frame_id << "," << cmd << ","
           << (int)command_status << "," << command_data_length << ",";

        if (command_data_length > 0)
        {
            for (uint16_t i = 0; i < command_data_length; i++)
            {
                os << (int)command_data[i] << " ";
            }
        }
        else
        {
            os << "-";
        }

        os << "\n";
    }
};

struct ModemStatusFrameLog
{
    long long timestamp  = 0;
    uint8_t modem_status = 0;

    static bool toFrameType(APIFrame& api, ModemStatusFrameLog* dest)
    {
        if (api.frame_type != FTYPE_MODEM_STATUS)
        {
            return false;
        }

        if (api.getFrameDataLength() != MODEM_STATUS_FRAME_SIZE)
        {
            return false;
        }

        ModemStatusFrame* modem = api.toFrameType<ModemStatusFrame>();

        dest->timestamp    = api.timestamp;
        dest->modem_status = modem->getStatus();

        return true;
    }

    static string header() { return "timestamp,status\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)modem_status << "\n";
    }
};

struct TXStatusFrameLog
{
    long long timestamp      = 0;
    uint8_t frame_id         = 0;
    uint8_t tx_retry_count   = 0;
    uint8_t delivery_status  = 0;
    uint8_t discovery_status = 0;

    static bool toFrameType(APIFrame& api, TXStatusFrameLog* dest)
    {
        if (api.frame_type != FTYPE_TX_STATUS)
        {
            return false;
        }

        if (api.getFrameDataLength() != TX_STATUS_FRAME_SIZE)
        {
            return false;
        }

        TXStatusFrame* tx = api.toFrameType<TXStatusFrame>();

        dest->timestamp = api.timestamp;
        dest->frame_id  = tx->getFrameID();

        dest->tx_retry_count   = tx->getTransmitRetryCount();
        dest->delivery_status  = tx->getDeliveryStatus();
        dest->discovery_status = tx->getDiscoveryStatus();

        return true;
    }

    static string header()
    {
        return "timestamp,id,tx_retries,delivery_status,discovery_status\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)frame_id << "," << (int)tx_retry_count
           << "," << (int)delivery_status << "," << (int)discovery_status
           << "\n";
    }
};

struct RXPacketFrameLog
{
    long long timestamp = 0;

    uint64_t source_address = 0;

    uint8_t receive_options = 0;

    uint8_t rx_data[MAX_PACKET_PAYLOAD_LENGTH];
    uint16_t rx_data_length = 0;

    static bool toFrameType(APIFrame& api, RXPacketFrameLog* dest)
    {
        if (api.frame_type != FTYPE_RX_PACKET_FRAME)
        {
            return false;
        }

        if (api.getFrameDataLength() < MIN_RX_PACKET_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_RX_PACKET_FRAME_SIZE + MAX_PACKET_PAYLOAD_LENGTH)
        {
            return false;
        }

        RXPacketFrame* rx = api.toFrameType<RXPacketFrame>();

        dest->timestamp      = api.timestamp;
        dest->source_address = rx->getSourceAddress();

        dest->receive_options = rx->getReceiveOptions();

        dest->rx_data_length = rx->getRXDataLength();
        memcpy(dest->rx_data, rx->getRXDataPointer(), dest->rx_data_length);

        return true;
    }

    static string header()
    {
        return "timestamp,src_addr,rx_options,rx_data_size,payload32_0,payload32_1\n";
    }

    void print(std::ostream& os) const
    {
        uint32_t data[2];
        memcpy(&data, rx_data, min(sizeof(uint32_t)*2, (size_t)rx_data_length));
        os << timestamp << "," << source_address << "," << (int)receive_options
           << "," << rx_data_length << "," << data[0] << "," << data[1] << "\n";
    }
};

}  // namespace Xbee