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

#include <interfaces/endianness.h>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <ostream>
#include <string>
#include <type_traits>
#include <utility>

using std::min;
using std::string;

namespace Boardcore
{

namespace Xbee
{
/// Maximum length for a TX/RX frame payload. User-configurable using the "NP"
/// AT command or the Xbee configuration software (see Xbee datasheet)
static constexpr size_t MIN_RX_PACKET_FRAME_SIZE   = 11;
static constexpr size_t MIN_TX_REQUEST_FRAME_SIZE  = 13;
static constexpr size_t MIN_AT_COMMAND_FRAME_SIZE  = 3;
static constexpr size_t MIN_AT_RESPONSE_FRAME_SIZE = 4;

static constexpr size_t TX_STATUS_FRAME_SIZE    = 6;
static constexpr size_t MODEM_STATUS_FRAME_SIZE = 1;

static constexpr uint16_t MAX_PACKET_PAYLOAD_LENGTH      = 256;
static constexpr uint16_t MAX_AT_COMMAND_PARAMS_LENGTH   = 20;
static constexpr uint16_t MAX_AT_COMMAND_RESPONSE_LENGTH = 30;

static constexpr size_t FRAME_DATA_SIZE =
    MAX_PACKET_PAYLOAD_LENGTH + MIN_TX_REQUEST_FRAME_SIZE;

static constexpr size_t MAX_API_FRAME_SIZE = FRAME_DATA_SIZE + 5;
static constexpr size_t MIN_API_FRAME_SIZE = 5;

static constexpr uint64_t ADDRESS_BROADCAST = 0xFFFF;
static constexpr uint8_t START_DELIMITER    = 0x7E;

enum FrameType : uint8_t
{
    FTYPE_AT_COMMAND          = 0x08,
    FTYPE_AT_COMMAND_QUEUE    = 0x09,
    FTYPE_TX_REQUEST          = 0x10,
    FTYPE_AT_COMMAND_RESPONSE = 0x88,
    FTYPE_MODEM_STATUS        = 0x8A,
    FTYPE_TX_STATUS           = 0x8B,
    FTYPE_RX_PACKET_FRAME     = 0x90,
};

enum TransmitOptionsBitfield : uint8_t
{
    TO_DISABLE_ACK = 0x01,
    TO_DISABLE_RD  = 0x02,
    TO_NACK        = 0x04,
    TO_TRACE_ROUTE = 0x08,

    TO_DM_POINT_MULTIPOINT = 0x40,
    TO_DM_REPEATER_MODE    = 0x80,
    TO_DM_DIGIMESH         = 0xC0
};

enum CommandStatusBitfield : uint8_t
{
    CS_OK                = 0x00,
    CS_ERROR             = 0x01,
    CS_INVALID_COMMAND   = 0x02,
    CS_INVALID_PARAMETER = 0x03,
    CS_RSSI_INVALID      = 0x40,
    CS_IS_REMOTE_COMMAND = 0x80
};

enum ModemStatus : uint8_t
{
    MS_HARDWARE_RESET       = 0x00,
    MS_WATCHDOG_TIMER_RESET = 0x01,
    MS_NETWORK_WOKE_UP      = 0x0B,
    MS_NETWORK_WENT_SLEEP   = 0x0C,
};

enum DeliveryStatus : uint8_t
{
    DELS_SUCCESS               = 0x00,
    DELS_MAC_ACK_FAILURE       = 0x01,
    DELS_COLL_AVOID_FAILURE    = 0x02,
    DELS_NO_SPECTRUM_AVAILABLE = 0x03,
    DELS_NET_ACK_FAILURE       = 0x21,
    DELS_ROUTE_NOT_FOUND       = 0x25,
    DELS_INT_RESOURCE_ERR      = 0x31,
    DELS_INTERNAL_ERROR        = 0x32,
    DELS_PAYLOAD_TOO_LARGE     = 0x74,
    DELS_INDIRECT_MSG_REQ      = 0x75,
};

enum DiscoveryStatus : uint8_t
{
    DISCS_NO_DISC_OVERHEAD = 0x00,
    DISCS_ROUTE_DISCOVERY  = 0x02
};

enum ReceiveOptions : uint8_t
{
    RO_PACKET_ACK          = 0x00,
    RO_PACKET_IS_BROADCAST = 0x01,
    RO_POINT_MULTIPOINT    = 0x40,
    RO_REPEATER_MODE       = 0x80,
    RO_DIGIMESH            = 0xC0,
};

#pragma pack(1)
struct APIFrame
{
    uint8_t startDel  = 0x7E;
    uint16_t length   = 0;
    uint8_t frameType = 0;
    uint8_t frameData[FRAME_DATA_SIZE];

    uint8_t checksum = 0;
    // Used for logging, not part of the standard Xbee API Frame
    long long timestamp = 0;

    APIFrame() { memset(frameData, 0, FRAME_DATA_SIZE); }

    uint16_t getFrameDataLength() const
    {
        size_t len = swapBytes16(length) - 1;

        return min(len, FRAME_DATA_SIZE);
    }

    void setFrameDataLength(uint16_t len)
    {
        length = swapBytes16(min((size_t)(len + 1), FRAME_DATA_SIZE + 1));
    }

    bool verifyChecksum() const
    {
        // Sum all the bytes including checksum and frame type.
        // The sum can be stored in a uint8_t since we only care about the least
        // significant byte.
        uint8_t sum = checksum + frameType;
        for (uint16_t i = 0; i < getFrameDataLength(); ++i)
        {
            sum += frameData[i];
        }
        return sum == 0xFF;
    }

    void calcChecksum()
    {
        checksum = frameType;
        for (uint16_t i = 0; i < getFrameDataLength(); ++i)
        {
            checksum += frameData[i];
        }

        checksum = 0xFF - checksum;
    }

    template <typename FrameType>
    FrameType* toFrameType()
    {
        static_assert(
            std::is_base_of<APIFrame, FrameType>::value ||
                std::is_same<APIFrame, FrameType>::value,
            "FrameType must be derived from APIFrame or be an APIFrame");

        return reinterpret_cast<FrameType*>(this);
    }

    size_t toBytes(uint8_t* bytes)
    {
        memcpy(bytes, this, 4 + getFrameDataLength());

        bytes[4 + getFrameDataLength()] = checksum;
        return 5 + getFrameDataLength();
    }

    static bool fromBytes(uint8_t* bytes, size_t size, APIFrame* f)
    {
        if (size >= MIN_API_FRAME_SIZE && size <= MAX_API_FRAME_SIZE)
        {
            memcpy(f, bytes, size - 1);
            f->checksum = bytes[size - 1];

            return true;
        }
        return false;
    }
};
#pragma pack()
struct ATCommandFrame : public APIFrame
{
    ATCommandFrame() : APIFrame()
    {
        frameType = FTYPE_AT_COMMAND;
        setFrameDataLength(MIN_AT_COMMAND_FRAME_SIZE);
    }

    uint8_t getFrameID() const { return frameData[0]; }

    void setFrameID(uint8_t frameId) { frameData[0] = frameId; }

    const char* getATCommand() const
    {
        return reinterpret_cast<const char*>(&frameData[1]);
    }

    void setATCommand(const char* at)
    {
        frameData[1] = at[0];
        frameData[2] = at[1];
    }

    uint8_t* getCommandDataPointer() { return &frameData[3]; }

    uint16_t getCommandDataLength() const
    {
        return getFrameDataLength() - MIN_AT_COMMAND_FRAME_SIZE;
    }

    void setParameterSize(uint16_t size)
    {
        size = min((size_t)size, FRAME_DATA_SIZE - MIN_AT_COMMAND_FRAME_SIZE);

        setFrameDataLength(MIN_AT_COMMAND_FRAME_SIZE + size);
    }
};
static_assert(sizeof(ATCommandFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");

struct ATCommandResponseFrame : public APIFrame
{
    ATCommandResponseFrame() : APIFrame()
    {
        frameType = FTYPE_AT_COMMAND_RESPONSE;
        setFrameDataLength(MIN_AT_RESPONSE_FRAME_SIZE);
    }

    uint8_t getFrameID() const { return frameData[0]; }

    void setFrameID(uint8_t frameId) { frameData[0] = frameId; }

    const char* getATCommand() const
    {
        return reinterpret_cast<const char*>(&frameData[1]);
    }

    void setATCommand(const char* at)
    {
        frameData[1] = at[0];
        frameData[2] = at[1];
    }

    uint8_t getCommandStatus() const { return frameData[3]; }

    void setCommandStatus(uint8_t cs) { frameData[3] = cs; }

    uint8_t* getCommandDataPointer() { return &frameData[4]; }

    uint16_t getCommandDataLength() const
    {
        return getFrameDataLength() - MIN_AT_RESPONSE_FRAME_SIZE;
    }

    void setCommandDataSize(uint16_t size)
    {
        size = min((size_t)size, FRAME_DATA_SIZE - MIN_AT_RESPONSE_FRAME_SIZE);

        setFrameDataLength(MIN_AT_RESPONSE_FRAME_SIZE + size);
    }
};
static_assert(sizeof(ATCommandFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");

struct TXRequestFrame : public APIFrame
{
    TXRequestFrame() : APIFrame()
    {
        frameType = FTYPE_TX_REQUEST;
        setFrameDataLength(MIN_TX_REQUEST_FRAME_SIZE);

        // Reserved bytes
        frameData[9]  = 0xFF;
        frameData[10] = 0xFE;
    }

    uint8_t getFrameID() const { return frameData[0]; }

    void setFrameID(uint8_t frameId) { frameData[0] = frameId; }

    uint64_t getDestAddress() const
    {
        uint64_t addr;
        memcpy(&addr, &frameData[1], sizeof(uint64_t));
        return swapBytes64(addr);
    }

    void setDestAddress(uint64_t address)
    {
        address = swapBytes64(address);
        memcpy(&frameData[1], &address, 8);
    }

    uint8_t getBroadcastRadius() const { return frameData[11]; }

    void setBroadcastRadius(uint8_t br) { frameData[11] = br; }

    uint8_t getTrasmitOptions() const { return frameData[12]; }

    void setTransmitOptions(uint8_t br) { frameData[12] = br; }

    uint8_t* getRFDataPointer() { return &frameData[13]; }

    uint16_t getRFDataLength() const
    {
        return min((size_t)(getFrameDataLength() - MIN_TX_REQUEST_FRAME_SIZE),
                   (size_t)MAX_PACKET_PAYLOAD_LENGTH);
    }

    void setRFDataLength(uint16_t size)
    {
        size = min(size, MAX_PACKET_PAYLOAD_LENGTH);

        setFrameDataLength(MIN_TX_REQUEST_FRAME_SIZE + size);
    }
};
static_assert(sizeof(TXRequestFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");

struct ModemStatusFrame : public APIFrame
{
    ModemStatusFrame() : APIFrame()
    {
        frameType = FTYPE_MODEM_STATUS;
        setFrameDataLength(MODEM_STATUS_FRAME_SIZE);
    }

    uint8_t getStatus() const { return frameData[0]; }

    void setStatus(uint8_t status) { frameData[0] = status; }
};
static_assert(sizeof(ModemStatusFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");

struct TXStatusFrame : public APIFrame
{
    TXStatusFrame() : APIFrame()
    {
        frameType = FTYPE_TX_STATUS;
        setFrameDataLength(TX_STATUS_FRAME_SIZE);

        // Reserved bytes
        frameData[1] = 0xFF;
        frameData[2] = 0xFE;
    }

    uint8_t getFrameID() const { return frameData[0]; }

    void setFrameID(uint8_t frameId) { frameData[0] = frameId; }

    uint8_t getTransmitRetryCount() const { return frameData[3]; }

    void setTransmitRetryCount(uint8_t trc) { frameData[3] = trc; }

    uint8_t getDeliveryStatus() const { return frameData[4]; }

    void setDeliveryStatus(uint8_t ds) { frameData[4] = ds; }

    uint8_t getDiscoveryStatus() const { return frameData[5]; }

    void setDiscoveryStatus(uint8_t ds) { frameData[5] = ds; }
};

static_assert(sizeof(TXStatusFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");

struct RXPacketFrame : public APIFrame
{
    RXPacketFrame() : APIFrame()
    {
        frameType = FTYPE_RX_PACKET_FRAME;
        setFrameDataLength(MIN_RX_PACKET_FRAME_SIZE);

        frameData[8] = 0xFF;
        frameData[9] = 0xFE;
    }

    uint64_t getSourceAddress() const
    {
        uint64_t addr;
        memcpy(&addr, &frameData[0], sizeof(uint64_t));
        return swapBytes64(addr);
    }

    void setSourceAddress(uint64_t address)
    {
        address       = swapBytes64(address);
        uint8_t* addr = reinterpret_cast<uint8_t*>(&address);

        memcpy(&frameData[0], addr, 8);
    }

    uint8_t getReceiveOptions() const { return frameData[10]; }

    void setReceiveOptions(uint8_t ro) { frameData[10] = ro; }

    uint8_t* getRXDataPointer() { return &frameData[11]; }

    uint16_t getRXDataLength() const
    {
        return min((size_t)(getFrameDataLength() - MIN_RX_PACKET_FRAME_SIZE),
                   (size_t)MAX_PACKET_PAYLOAD_LENGTH);
    }

    void setRXDataLength(uint16_t size)
    {
        size = min(size, MAX_PACKET_PAYLOAD_LENGTH);

        setFrameDataLength(MIN_RX_PACKET_FRAME_SIZE + size);
    }
};

static_assert(sizeof(RXPacketFrame) == sizeof(APIFrame),
              "Size of derived classes must be the same as APIFrame class (no "
              "additional members & no virtual functions)");
}  // namespace Xbee

}  // namespace Boardcore
