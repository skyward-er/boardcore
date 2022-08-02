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
    OPCODE_DEVICE_DEVICEASYNCDUMMY              = 0x0063,
    OPCODE_DEVICE_DEVICEGETRESPONSE             = 0x0466,
    OPCODE_DEVICE_DEVICESETRESPONSE             = 0x04B7,
    OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT = 0x089A,  //< ????
    OPCODE_NETAPP_IPACQUIRED                    = 0x1825,  //< ????
    OPCODE_DEVICE_DEVICEGET                     = 0x8466,
    OPCODE_DEVICE_DEVICESET                     = 0x84B7,
};

//! Is this message synchronous?
inline bool isSync(OpCode op) { return op & OPCODE_SYNC; }

struct SyncPattern
{
    uint32_t long1;
    uint16_t short1;
    uint8_t byte1;
    uint8_t byte2;
};

struct DeviceVersion
{
    uint32_t chip_id;
    uint8_t fw_version[4];
    uint8_t phy_version[4];
    uint8_t nwp_version[4];
    uint16_t rom_version;
    uint16_t _pad;
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

struct DeviceSetGet
{
    uint16_t status;
    uint16_t device_set_id;
    uint16_t option;
    uint16_t config_len;
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

inline const char *opToStr(OpCode op)
{
    switch (op)
    {
        case OpCode::OPCODE_DEVICE_INITCOMPLETE:
            return "OPCODE_DEVICE_INITCOMPLETE";
        case OpCode::OPCODE_DEVICE_DEVICEASYNCDUMMY:
            return "OPCODE_DEVICE_DEVICEASYNCDUMMY";
        case OpCode::OPCODE_DEVICE_DEVICEGETRESPONSE:
            return "OPCODE_DEVICE_DEVICEGETRESPONSE";
        case OpCode::OPCODE_DEVICE_DEVICESETRESPONSE:
            return "OPCODE_DEVICE_DEVICESETRESPONSE";
        case OpCode::OPCODE_NETAPP_IPACQUIRED:
            return "OPCODE_NETAPP_IPACQUIRED";
        case OpCode::OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT:
            return "OPCODE_WLAN_PROVISIONING_STATUS_ASYNC_EVENT";
        case OpCode::OPCODE_DEVICE_DEVICEGET:
            return "OPCODE_DEVICE_DEVICEGET";
        case OpCode::OPCODE_DEVICE_DEVICESET:
            return "OPCODE_DEVICE_DEVICESET";
        default:
            return "<unknown>";
    }
}

}  // namespace CC3135Defs

}  // namespace Boardcore
