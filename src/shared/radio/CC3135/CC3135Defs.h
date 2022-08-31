/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <cstdlib>

/*
CC3135 Protocol definitions

This definitions come mostly from the TI driver and a bit of RE
*/

namespace Boardcore
{

namespace CC3135Defs
{

//! Synchronous message mask.
constexpr uint16_t OPCODE_SYNC = 1 << 10;

//! Device command opcodes.
enum OpCode : uint16_t
{
    OPCODE_DEVICE_INITCOMPLETE                  = 0x0008,
    OPCODE_DEVICE_ABORT                         = 0x000C,
    OPCODE_DEVICE_DEVICEASYNCDUMMY              = 0x0063,
    OPCODE_DEVICE_DEVICE_ASYNC_GENERAL_ERROR    = 0x0078,
    OPCODE_DEVICE_DEVICEGETRESPONSE             = 0x0466,
    OPCODE_DEVICE_DEVICESETRESPONSE             = 0x04B7,
    OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT = 0x089A,  //< ????
    OPCODE_WLAN_WLANDISCONNECTRESPONSE          = 0x0C81,
    OPCODE_WLAN_POLICYSETRESPONSE               = 0x0C86,
    OPCODE_WLAN_SET_MODE_RESPONSE               = 0x0CB4,
    OPCODE_WLAN_CFG_SET_RESPONSE                = 0x0CB5,
    OPCODE_SOCKET_RECVASYNCRESPONSE             = 0x100A,
    OPCODE_SOCKET_SOCKETRESPONSE                = 0x1401,
    OPCODE_SOCKET_CLOSERESPONSE                 = 0x1402,
    OPCODE_NETAPP_IPACQUIRED                    = 0x1825,  //< ????
    OPCODE_DEVICE_DEVICEGET                     = 0x8466,
    OPCODE_DEVICE_DEVICESET                     = 0x84B7,
    OPCODE_WLAN_POLICYSETCOMMAND                = 0x8C86,
    OPCODE_WLAN_WLANDISCONNECTCOMMAND           = 0x8C81,
    OPCODE_WLAN_SET_MODE                        = 0x8CB4,
    OPCODE_WLAN_CFG_SET                         = 0x8CB5,
    OPCODE_SOCKET_SOCKET                        = 0x9401,
    OPCODE_SOCKET_CLOSE                         = 0x9402,
    OPCODE_SOCKET_RECV                          = 0x940A,
    OPCODE_SOCKET_SEND                          = 0x940C,
};

//! Is this message synchronous?
inline bool isSync(OpCode op) { return op & OPCODE_SYNC; }

inline const char *opToStr(OpCode op)
{
    switch (op)
    {
        case OpCode::OPCODE_DEVICE_INITCOMPLETE:
            return "OPCODE_DEVICE_INITCOMPLETE";
        case OpCode::OPCODE_DEVICE_ABORT:
            return "OPCODE_DEVICE_ABORT";
        case OpCode::OPCODE_DEVICE_DEVICEASYNCDUMMY:
            return "OPCODE_DEVICE_DEVICEASYNCDUMMY";
        case OpCode::OPCODE_DEVICE_DEVICE_ASYNC_GENERAL_ERROR:
            return "OPCODE_DEVICE_DEVICE_ASYNC_GENERAL_ERROR";
        case OpCode::OPCODE_DEVICE_DEVICEGETRESPONSE:
            return "OPCODE_DEVICE_DEVICEGETRESPONSE";
        case OpCode::OPCODE_DEVICE_DEVICESETRESPONSE:
            return "OPCODE_DEVICE_DEVICESETRESPONSE";
        case OpCode::OPCODE_NETAPP_IPACQUIRED:
            return "OPCODE_NETAPP_IPACQUIRED";
        case OpCode::OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT:
            return "OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT";
        case OpCode::OPCODE_WLAN_WLANDISCONNECTRESPONSE:
            return "OPCODE_WLAN_WLANDISCONNECTRESPONSE";
        case OpCode::OPCODE_WLAN_POLICYSETRESPONSE:
            return "OPCODE_WLAN_POLICYSETRESPONSE";
        case OpCode::OPCODE_WLAN_SET_MODE_RESPONSE:
            return "OPCODE_WLAN_SET_MODE_RESPONSE";
        case OpCode::OPCODE_WLAN_CFG_SET_RESPONSE:
            return "OPCODE_WLAN_CFG_SET_RESPONSE";
        case OpCode::OPCODE_SOCKET_RECVASYNCRESPONSE:
            return "OPCODE_SOCKET_RECVASYNCRESPONSE";
        case OpCode::OPCODE_SOCKET_SOCKETRESPONSE:
            return "OPCODE_SOCKET_SOCKETRESPONSE";
        case OpCode::OPCODE_SOCKET_CLOSERESPONSE:
            return "OPCODE_SOCKET_CLOSERESPONSE";
        case OpCode::OPCODE_DEVICE_DEVICEGET:
            return "OPCODE_DEVICE_DEVICEGET";
        case OpCode::OPCODE_DEVICE_DEVICESET:
            return "OPCODE_DEVICE_DEVICESET";
        case OpCode::OPCODE_WLAN_POLICYSETCOMMAND:
            return "OPCODE_WLAN_POLICYSETCOMMAND";
        case OpCode::OPCODE_WLAN_WLANDISCONNECTCOMMAND:
            return "OPCODE_WLAN_WLANDISCONNECTCOMMAND";
        case OpCode::OPCODE_WLAN_SET_MODE:
            return "OPCODE_WLAN_SET_MODE";
        case OpCode::OPCODE_WLAN_CFG_SET:
            return "OPCODE_WLAN_CFG_SET";
        case OpCode::OPCODE_SOCKET_SOCKET:
            return "OPCODE_SOCKET_SOCKET";
        case OpCode::OPCODE_SOCKET_CLOSE:
            return "OPCODE_SOCKET_CLOSE";
        case OpCode::OPCODE_SOCKET_RECV:
            return "OPCODE_SOCKET_RECV";
        case OpCode::OPCODE_SOCKET_SEND:
            return "OPCODE_SOCKET_SEND";
        default:
            return "<unknown>";
    }
}

enum Mode : uint8_t
{
    ROLE_STA      = 0,
    ROLE_RESERVED = 1,
    ROLE_AP       = 2,
    ROLE_P2P      = 3,
    ROLE_TAG      = 4
};

struct DeviceVersion
{
    uint32_t chip_id;
    uint8_t fw_version[4];
    uint8_t phy_version[4];
    uint8_t nwp_version[4];
    uint16_t rom_version;
    uint8_t _pad[2];
};

struct GenericHeader
{
    OpCode opcode;
    uint16_t len;
};

struct ResponseHeader
{
    GenericHeader inner;
    uint8_t tx_pool_count;
    uint8_t dev_status;
    uint16_t min_max_payload;
    uint16_t socket_tx_failure;
    uint16_t socket_non_blocking;
};

typedef GenericHeader RequestHeader;

struct BasicResponse
{
    int16_t status;
    uint16_t sender;
};

struct DeviceInitInfo
{
    int32_t status;
    int32_t chip_id;
    int32_t more_data;
};

struct DeviceSetGet
{
    uint16_t status;
    uint16_t device_set_id;
    uint16_t option;
    uint16_t config_len;
};

constexpr uint16_t DEVICE_GENERAL         = 1;
constexpr uint16_t DEVICE_GENERAL_VERSION = 12;

constexpr uint16_t DEVICE_STATUS              = 2;
constexpr uint16_t DEVICE_EVENT_CLASS_DEVICE  = 1;
constexpr uint16_t DEVICE_EVENT_CLASS_WLAN    = 2;
constexpr uint16_t DEVICE_EVENT_CLASS_BSD     = 3;
constexpr uint16_t DEVICE_EVENT_CLASS_NETAPP  = 4;
constexpr uint16_t DEVICE_EVENT_CLASS_NETCFG  = 5;
constexpr uint16_t DEVICE_EVENT_CLASS_FS      = 6;
constexpr uint16_t DEVICE_EVENT_CLASS_NETUTIL = 7;

struct WlanCfgSetGet
{
    uint16_t status;
    uint16_t config_id;
    uint16_t option;
    uint16_t config_len;
};

constexpr uint16_t WLAN_CFG_AP_ID      = 0;
constexpr uint16_t WLAN_AP_OPT_CHANNEL = 3;

struct WlanSetMode
{
    Mode mode;
    uint8_t _pad[3];
};

struct WlanPolicySetGet
{
    uint8_t policy_type;
    uint8_t _pad;
    uint8_t policy_option;
    uint8_t policy_option_len;
};

constexpr uint8_t WLAN_POLICY_CONNECTION = 16;

/// Socket descriptor;
using Sd = uint8_t;

enum class SdType
{
    RAW_TRANSCEIVER,
    GENERIC
};

inline SdType sdGetType(Sd sd)
{
    if ((sd & 0xf0) == 0x80)
    {
        return SdType::RAW_TRANSCEIVER;
    }
    else
    {
        return SdType::GENERIC;
    }
}

struct SocketCommand
{
    uint8_t domain;
    uint8_t type;
    uint8_t protocol;
    uint8_t padding;
};

constexpr uint8_t AF_INET  = 2;
constexpr uint8_t AF_INET6 = 3;
constexpr uint8_t AF_RF    = 6;

constexpr uint8_t SOCK_STREAM = 1;
constexpr uint8_t SOCK_DGRAM  = 2;
constexpr uint8_t SOCK_RAW    = 3;
constexpr uint8_t SOCK_RX_MTR = 4;

constexpr uint8_t IPPROTO_TCP = 6;
constexpr uint8_t IPPROTO_UDP = 17;

struct CloseCommand
{
    Sd sd;
    uint8_t _pad[3];
};

struct SendRecvCommand
{
    uint16_t status_or_len;
    Sd sd;
    uint8_t family_and_flags;
};

struct SendRecvCommand2
{
    uint16_t status_or_len;
    Sd sd;
    uint8_t family_and_flags;
    uint16_t flags;
    uint8_t _pad[2];
};

struct SocketResponse
{
    int16_t status_or_len;
    Sd sd;
    uint8_t _pad;
};

enum Rate : uint16_t
{
    RATE_1M    = 1,
    RATE_2M    = 2,
    RATE_5_5M  = 3,
    RATE_11M   = 4,
    RATE_6M    = 6,
    RATE_9M    = 7,
    RATE_12M   = 8,
    RATE_18M   = 9,
    RATE_24M   = 10,
    RATE_36M   = 11,
    RATE_48M   = 12,
    RATE_54M   = 13,
    RATE_MCS_0 = 14,
    RATE_MCS_1 = 15,
    RATE_MCS_2 = 16,
    RATE_MCS_3 = 17,
    RATE_MCS_4 = 18,
    RATE_MCS_5 = 19,
    RATE_MCS_6 = 20,
    RATE_MCS_7 = 21,
};

enum Preable : uint16_t
{
    PREAMBLE_LONG  = 0,
    PREAMBLE_SHORT = 1
};

inline uint16_t makeWlanRawRfTxParams(uint8_t channel, Rate rate, uint8_t power,
                                      Preable preamble)
{
    constexpr uint8_t MAX_2_4G_CHANNEL = 14;
    constexpr uint8_t BAND_2_4G        = 0;
    constexpr uint8_t BAND_5_0G        = 1;

    constexpr uint8_t CHANNEL_SHIFT  = 0;
    constexpr uint8_t BAND_SHIFT     = 5;
    constexpr uint8_t RATE_SHIFT     = 6;
    constexpr uint8_t POWER_SHIFT    = 11;
    constexpr uint8_t PREAMBLE_SHIFT = 15;

    constexpr uint8_t CHANNEL_5G_SHIFT = 12;

    if (channel <= MAX_2_4G_CHANNEL)
    {
        return ((channel << CHANNEL_SHIFT) | (BAND_2_4G << BAND_SHIFT) |
                (rate << RATE_SHIFT) | (power << POWER_SHIFT) |
                (preamble << PREAMBLE_SHIFT));
    }
    else
    {
        uint8_t channel_lo = channel & 0x1f;
        uint8_t channel_hi = (channel & 0xe0) >> 5;

        return ((channel_lo << CHANNEL_SHIFT) | (BAND_5_0G << BAND_SHIFT) |
                (rate << RATE_SHIFT) | (channel_hi << CHANNEL_5G_SHIFT) |
                (power << POWER_SHIFT) | (preamble << PREAMBLE_SHIFT));
    }
}

// Stuff required for synchronization

struct SyncPattern
{
    uint32_t long1;
    uint16_t short1;
    uint8_t byte1;
    uint8_t byte2;
};

constexpr SyncPattern H2N_SYNC_PATTERN = {0xBBDDEEFF, 0x4321, 0x34, 0x12};
constexpr SyncPattern H2N_CNYS_PATTERN = {0xBBDDEEFF, 0x8765, 0x78, 0x56};

constexpr size_t SYNC_PATTERN_LEN = sizeof(uint32_t);

constexpr uint32_t N2H_SYNC_PATTERN                = 0xABCDDCBA;
constexpr uint32_t N2H_SYNC_PATTERN_SEQ_NUM_BITS   = 0x00000003;
constexpr uint32_t N2H_SYNC_PATTERN_SEQ_NUM_EXISTS = 0x00000004;
constexpr uint32_t N2H_SYNC_PATTERN_MASK           = 0xFFFFFFF8;
constexpr uint32_t N2H_SYNC_SPI_BUGS_MASK          = 0x7FFF7F7F;  // What?

inline bool n2hSyncPatternMatch(uint32_t sync, uint8_t seq_num)
{
    // TODO: Add sequence number
    return (sync & N2H_SYNC_SPI_BUGS_MASK & N2H_SYNC_PATTERN_MASK) ==
           (N2H_SYNC_PATTERN & N2H_SYNC_SPI_BUGS_MASK & N2H_SYNC_PATTERN_MASK);
}

//! Align message size.
inline size_t alignSize(size_t size) { return (size + 3) & (~3); }

}  // namespace CC3135Defs

}  // namespace Boardcore
