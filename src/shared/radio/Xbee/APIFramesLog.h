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

namespace Boardcore
{

/**
 * Classes used if you want to log an APIFrame, but do not want to log the base
 * APIFrame class, which is quite heavy on memory
 */

namespace Xbee
{

struct APIFrameLog
{
    int64_t timestamp;
    uint8_t frameType;
    uint16_t frameDataLength;
    uint8_t frameData[FRAME_DATA_SIZE];

    static bool fromAPIFrame(APIFrame& api, APIFrameLog* dest)
    {
        dest->timestamp       = api.timestamp;
        dest->frameType       = api.frameType;
        dest->frameDataLength = api.getFrameDataLength();

        memcpy(dest->frameData, api.frameData, dest->frameDataLength);

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(APIFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(frameType)
                              FIELD_DEF(frameDataLength) FIELD_DEF(frameData));
    }
};

struct ATCommandFrameLog
{
    int32_t timestamp;
    uint8_t frameId = 0;
    char atCommand[2];

    uint8_t commandData[MAX_AT_COMMAND_PARAMS_LENGTH];
    uint16_t commandDataLength = 0;

    static bool toFrameType(APIFrame& api, ATCommandFrameLog* dest)
    {
        if (api.frameType != FTYPE_AT_COMMAND)
            return false;

        if (api.getFrameDataLength() < MIN_AT_COMMAND_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_AT_COMMAND_FRAME_SIZE + MAX_AT_COMMAND_PARAMS_LENGTH)
        {
            return false;
        }
        ATCommandFrame* at = api.toFrameType<ATCommandFrame>();

        dest->timestamp = at->timestamp;
        dest->frameId   = at->getFrameID();

        memcpy(dest->atCommand, at->getATCommand(), 2);

        dest->commandDataLength = at->getCommandDataLength();
        memcpy(dest->commandData, at->getCommandDataPointer(),
               dest->commandDataLength);

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ATCommandFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(frameId)
                              FIELD_DEF(atCommand) FIELD_DEF(commandData)
                                  FIELD_DEF(commandDataLength));
    }
};

struct TXRequestFrameLog
{
    int64_t timestamp = 0;
    uint8_t frameId   = 0;

    uint64_t destinationAddress = 0;

    uint8_t broadcastRadius = 0;
    uint8_t transmitOptions = 0;

    uint8_t rfData[MAX_PACKET_PAYLOAD_LENGTH];
    uint16_t rfDataLength = 0;

    static bool toFrameType(APIFrame& api, TXRequestFrameLog* dest)
    {
        if (api.frameType != FTYPE_TX_REQUEST)
            return false;

        if (api.getFrameDataLength() < MIN_TX_REQUEST_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_TX_REQUEST_FRAME_SIZE + MAX_PACKET_PAYLOAD_LENGTH)
        {
            return false;
        }

        TXRequestFrame* tx = api.toFrameType<TXRequestFrame>();

        dest->timestamp = api.timestamp;
        dest->frameId   = tx->getFrameID();

        dest->destinationAddress = tx->getDestAddress();

        dest->broadcastRadius = tx->getBroadcastRadius();
        dest->transmitOptions = tx->getTrasmitOptions();

        dest->rfDataLength = tx->getRFDataLength();
        memcpy(dest->rfData, tx->getRFDataPointer(), dest->rfDataLength);

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TXRequestFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(frameId) FIELD_DEF(
                              destinationAddress) FIELD_DEF(broadcastRadius)
                              FIELD_DEF(transmitOptions) FIELD_DEF(rfData)
                                  FIELD_DEF(rfDataLength));
    }
};

struct ATCommandResponseFrameLog
{
    int64_t timestamp = 0;
    uint8_t frameId   = 0;
    char atCommand[2];
    uint8_t commandStatus = 0;

    uint8_t commandData[MAX_AT_COMMAND_RESPONSE_LENGTH];
    uint16_t commandDataLength = 0;

    static bool toFrameType(APIFrame& api, ATCommandResponseFrameLog* dest)
    {
        if (api.frameType != FTYPE_AT_COMMAND_RESPONSE)
            return false;

        if (api.getFrameDataLength() < MIN_AT_COMMAND_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_AT_COMMAND_FRAME_SIZE + MAX_AT_COMMAND_RESPONSE_LENGTH)
        {
            return false;
        }

        ATCommandResponseFrame* at = api.toFrameType<ATCommandResponseFrame>();

        dest->timestamp = api.timestamp;
        dest->frameId   = at->getFrameID();
        memcpy(dest->atCommand, at->getATCommand(), 2);

        dest->commandStatus = at->getCommandStatus();

        dest->commandDataLength = at->getCommandDataLength();
        memcpy(dest->commandData, at->getCommandDataPointer(),
               dest->commandDataLength);

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ATCommandResponseFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(frameId)
                              FIELD_DEF(atCommand) FIELD_DEF(commandStatus)
                                  FIELD_DEF(commandData)
                                      FIELD_DEF(commandDataLength));
    }
};

struct ModemStatusFrameLog
{
    int64_t timestamp   = 0;
    uint8_t modemStatus = 0;

    static bool toFrameType(APIFrame& api, ModemStatusFrameLog* dest)
    {
        if (api.frameType != FTYPE_MODEM_STATUS)
            return false;

        if (api.getFrameDataLength() != MODEM_STATUS_FRAME_SIZE)
            return false;

        ModemStatusFrame* modem = api.toFrameType<ModemStatusFrame>();

        dest->timestamp   = api.timestamp;
        dest->modemStatus = modem->getStatus();

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ModemStatusFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(modemStatus));
    }
};

struct TXStatusFrameLog
{
    int64_t timestamp       = 0;
    uint8_t frameId         = 0;
    uint8_t txRetryCount    = 0;
    uint8_t deliveryStatus  = 0;
    uint8_t discoveryStatus = 0;

    static bool toFrameType(APIFrame& api, TXStatusFrameLog* dest)
    {
        if (api.frameType != FTYPE_TX_STATUS)
            return false;

        if (api.getFrameDataLength() != TX_STATUS_FRAME_SIZE)
            return false;

        TXStatusFrame* tx = api.toFrameType<TXStatusFrame>();

        dest->timestamp = api.timestamp;
        dest->frameId   = tx->getFrameID();

        dest->txRetryCount    = tx->getTransmitRetryCount();
        dest->deliveryStatus  = tx->getDeliveryStatus();
        dest->discoveryStatus = tx->getDiscoveryStatus();

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TXStatusFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(frameId)
                              FIELD_DEF(txRetryCount) FIELD_DEF(deliveryStatus)
                                  FIELD_DEF(discoveryStatus));
    }
};

struct RXPacketFrameLog
{
    int64_t timestamp = 0;

    uint64_t sourceAddress = 0;

    uint8_t receiveOptions = 0;

    uint8_t rxData[MAX_PACKET_PAYLOAD_LENGTH];
    uint16_t rxDataLength = 0;

    static bool toFrameType(APIFrame& api, RXPacketFrameLog* dest)
    {
        if (api.frameType != FTYPE_RX_PACKET_FRAME)
            return false;

        if (api.getFrameDataLength() < MIN_RX_PACKET_FRAME_SIZE ||
            api.getFrameDataLength() >
                MIN_RX_PACKET_FRAME_SIZE + MAX_PACKET_PAYLOAD_LENGTH)
        {
            return false;
        }

        RXPacketFrame* rx = api.toFrameType<RXPacketFrame>();

        dest->timestamp     = api.timestamp;
        dest->sourceAddress = rx->getSourceAddress();

        dest->receiveOptions = rx->getReceiveOptions();

        dest->rxDataLength = rx->getRXDataLength();
        memcpy(dest->rxData, rx->getRXDataPointer(), dest->rxDataLength);

        return true;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(RXPacketFrameLog,
                          FIELD_DEF(timestamp) FIELD_DEF(sourceAddress)
                              FIELD_DEF(receiveOptions) FIELD_DEF(rxData)
                                  FIELD_DEF(rxDataLength));
    }
};

}  // namespace Xbee

}  // namespace Boardcore
