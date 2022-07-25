/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include "CC3135.h"

namespace Boardcore
{

using namespace CC3135Defs;

void CC3135Proto::handleIntr()
{
    // TODO:
}

void CC3135::dummyRead()
{
    uint8_t buf[220];
    proto.readPacket(buf);

    ResponseHeader header;
    memcpy(&header, &buf[0], sizeof(ResponseHeader));

    // TRACE("status: %d\n", header.dev_status);
    // TRACE("opcode: %s\n", opToStr(header.gen_header.opcode));
    // TRACE("len: %d\n", header.gen_header.len);
    // TRACE("\n");

    // if(header.gen_header.opcode == OPCODE_DEVICE_DEVICEGETRESPONSE) {
    //     for(int i = 0; i < header.gen_header.len + 4; i++) {
    //         TRACE("%2x\n", buf[i]);
    //     }
    // }
}

DeviceVersion CC3135::getVersion()
{
    DeviceSetGet packet;
    packet.gen_header.opcode = OPCODE_DEVICE_DEVICEGET;
    packet.device_set_id     = 1;   // Device get general
    packet.option            = 12;  // Device get version

    proto.writePacket((uint8_t *)&packet, sizeof(packet));

    return {};
}

//! Get a generic header out of a buffer.
GenericHeader *getHeader(uint8_t *buf) { return (GenericHeader *)(buf); }

void CC3135Proto::readPacket(uint8_t *buf)
{
    // 1. Write CNYS pattern (only if SPI)
    if (iface->is_spi())
    {
        SyncPattern sync = H2N_CNYS_PATTERN;
        iface->write((uint8_t *)(&sync.short1), SYNC_PATTERN_LEN);
    }

    // 2. Read initial data from the device
    iface->read(&buf[0], 8);

    /*
    Here the TI driver does some weird stuff.
    Also the TI comment is wrong, soooo, I'll just skip that part
    */

    // 3. Scan for device SYNC
    while (true)
    {
        // Try and find the SYNC here
        uint32_t sync;
        memcpy(&sync, &buf[0], 4);

        if (n2hSyncPatternMatch(sync, tx_seq_num))
        {
            // Copy the bytes after the sync to the start
            memcpy(&buf[0], &buf[4], 4);
            break;
        }

        // TODO: The TI driver reads 4 bytes at a time, is this also good?

        // Shift everything
        memmove(&buf[0], &buf[1], 7);
        iface->read(&buf[7], 1);
    }

    // 4. Scan for double syncs
    while (true)
    {
        uint32_t sync;
        memcpy(&sync, &buf[0], 4);

        if (!n2hSyncPatternMatch(sync, tx_seq_num))
        {
            break;
        }

        iface->read(&buf[0], 4);
    }

    tx_seq_num++;

    // 5. Parse generic header
    GenericHeader *header = getHeader(&buf[0]);

    // 6. Finalize and read rest of the data
    if (header->len > 0)
    {
        // TODO: The TI driver reads a ResponseHeader, violating zero size

        size_t aligned_len = alignSize(header->len);
        iface->read(&buf[sizeof(GenericHeader)], aligned_len);
    }
}

void CC3135Proto::writePacket(uint8_t *buf, size_t size)
{
    // 1. Write SYNC pattern
    if (iface->is_spi())
    {
        // Short pattern for SPI
        SyncPattern sync = H2N_SYNC_PATTERN;
        iface->write((uint8_t *)&sync.short1, SYNC_PATTERN_LEN);
    }
    else
    {
        // Long pattern for UART
        SyncPattern sync = H2N_SYNC_PATTERN;
        iface->write((uint8_t *)&sync.long1, SYNC_PATTERN_LEN * 2);
    }

    // 2. Setup header length
    GenericHeader *header = getHeader(&buf[0]);
    header->len           = size - sizeof(GenericHeader);

    // 3. Write message
    iface->write(buf, size);
}

}  // namespace Boardcore
