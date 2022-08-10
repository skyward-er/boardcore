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

#include "CC3135.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/Debug.h>

using namespace Boardcore::CC3135Defs;
using namespace miosix;

namespace Boardcore
{

CC3135::CC3135(std::unique_ptr<ICC3135Iface> &&iface) : iface(std::move(iface))
{
    // Start internal thread
    this->start();
    irq_wait_thread = thread;
}

CC3135Defs::DeviceVersion CC3135::getVersion()
{
    DeviceSetGet tx_command  = {};
    tx_command.device_set_id = 1;   // Device get general
    tx_command.option        = 12;  // Device get general version

    DeviceSetGet rx_command  = {};
    DeviceVersion rx_payload = {};

    inoutPacketSync(OPCODE_DEVICE_DEVICEGET, Buffer::from(&tx_command),
                    Buffer::null(), OPCODE_DEVICE_DEVICEGETRESPONSE,
                    Buffer::from(&rx_command), Buffer::from(&rx_payload));

    return rx_payload;
}

void CC3135::handleIrq()
{
    irq_count++;

    if (irq_wait_thread)
    {
        irq_wait_thread->IRQwakeup();
        if (irq_wait_thread->IRQgetPriority() >
            Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            Scheduler::IRQfindNextThread();
        }
    }
}

void CC3135::waitForIrq()
{
    Thread *cur = Thread::getCurrentThread();

    FastInterruptDisableLock dLock;
    while (irq_wait_thread != cur || irq_count == 0)
    {
        irq_wait_thread->IRQwait();
        {
            FastInterruptEnableLock eLock(dLock);
            Thread::yield();
        }
    }

    irq_count--;
}

void CC3135::installAsServiceThread()
{
    FastInterruptDisableLock dLock;
    irq_wait_thread = Thread::getCurrentThread();
}

void CC3135::restoreDefaultServiceThread()
{
    FastInterruptDisableLock dLock;
    irq_wait_thread = thread;

    // Wakeup just in case
    irq_wait_thread->IRQwakeup();
    if (irq_wait_thread->IRQgetPriority() >
        Thread::IRQgetCurrentThread()->IRQgetPriority())
    {
        Scheduler::IRQfindNextThread();
    }
}

void CC3135::defaultPacketHandler(CC3135Defs::ResponseHeader header)
{
    TRACE(
        "[cc3135] Received packet:\n"
        "- Opcode: %s (%4x)\n"
        "- Status: %u\n"
        "- Socket tx failures: %u\n"
        "- Socket non blocking: %u\n",
        opToStr(header.inner.opcode), header.inner.opcode, header.dev_status,
        header.socket_tx_failure, header.socket_non_blocking);

    // Dummy read rest of the data
    // TODO: Add async commands
    dummyRead(header.inner.len);
}

void CC3135::run()
{
    // TODO: Implement a way to stop this thread

    while (true)
    {
        waitForIrq();

        {
            // Lock the device interface
            Lock<FastMutex> lock(iface_mutex);

            ResponseHeader header;
            readHeader(&header);

            defaultPacketHandler(header);
        }
    }
}

void CC3135::inoutPacketSync(CC3135Defs::OpCode tx_opcode,
                             CC3135::Buffer tx_command,
                             CC3135::Buffer tx_payload,
                             CC3135Defs::OpCode rx_opcode,
                             CC3135::Buffer rx_command,
                             CC3135::Buffer rx_payload)
{
    installAsServiceThread();

    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    writePacket(tx_opcode, tx_command, tx_payload);

    readPacket(rx_opcode, rx_command, rx_payload);

    restoreDefaultServiceThread();
}

void CC3135::readPacketSync(CC3135Defs::OpCode opcode, CC3135::Buffer command,
                            CC3135::Buffer payload)
{
    installAsServiceThread();

    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    readPacket(opcode, command, payload);

    restoreDefaultServiceThread();
}

void CC3135::writePacketSync(CC3135Defs::OpCode opcode, CC3135::Buffer command,
                             CC3135::Buffer payload)
{
    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    writePacket(opcode, command, payload);
}

void CC3135::readPacket(OpCode opcode, CC3135::Buffer command,
                        CC3135::Buffer payload)
{
    while (true)
    {
        waitForIrq();

        // Locking the interface is not needed

        ResponseHeader header = {};
        readHeader(&header);

        if (header.inner.opcode != opcode)
        {
            defaultPacketHandler(header);
        }
        else
        {
            // Read the rest of the packet
            size_t len = header.inner.len;

            iface->read(command.ptr, std::min(len, command.len));
            len -= std::min(len, command.len);

            iface->read(payload.ptr, std::min(len, payload.len));
            len -= std::min(len, payload.len);

            // Read tail of remanining data
            if (len > 0)
                dummyRead(len);

            break;
        }
    }
}

void CC3135::writePacket(OpCode opcode, CC3135::Buffer command,
                         CC3135::Buffer payload)
{
    RequestHeader header{opcode,
                         static_cast<uint16_t>(command.len + payload.len)};

    writeHeader(&header);

    iface->write(command.ptr, command.len);
    iface->write(payload.ptr, payload.len);
}

void CC3135::readHeader(ResponseHeader *header)
{
    // 1. Write CNYS pattern (only if SPI)
    if (iface->is_spi())
    {
        SyncPattern sync = H2N_CNYS_PATTERN;
        iface->write(reinterpret_cast<uint8_t *>(&sync.short1),
                     SYNC_PATTERN_LEN);
    }

    uint8_t buf[sizeof(ResponseHeader)];

    // 2. Read initial data from the device
    iface->read(&buf[0], 8);

    /*
    Here the TI driver does some weird stuff.
    Also the TI comment is wrong, soooo, I'll just skip that part
    */

    // 3. Scan for device SYNC
    while (true)
    {
        // Reads in SPI are always more than 4 bytes

        bool found = false;
        int i;
        for (i = 0; i < 4; i++)
        {
            // Try and find the SYNC here
            uint32_t sync;
            memcpy(&sync, &buf[i], 4);

            if (sync == 0x0a7b2d01)
            {
                // BRUH
            }

            if (n2hSyncPatternMatch(sync, tx_seq_num))
            {
                found = true;
                break;
            }
        }

        // Shift everything and read more data
        if (found)
        {
            memmove(&buf[0], &buf[4 + i], 4 - i);
            iface->read(&buf[4 - i], 4 + i);
            break;
        }
        else
        {
            memmove(&buf[0], &buf[4], 4);
            iface->read(&buf[4], 4);
        }
    }

    // 4. Scan for double syncs
    /*uint32_t sync;
    memcpy(&sync, &buf[0], 4);
    while(n2hSyncPatternMatch(sync, tx_seq_num)) {
        memmove(&buf[0], &buf[4], 4);
        iface->read(&buf[4], 4);

        memcpy(&sync, &buf[0], 4);
    }*/

    tx_seq_num++;

    // 5. Read rest of the header
    iface->read(&buf[8], sizeof(ResponseHeader) - 8);
    memcpy(header, &buf[0], sizeof(ResponseHeader));

    // 6. Adjust for bigger response header
    header->inner.len -= sizeof(ResponseHeader) - sizeof(GenericHeader);
}

void CC3135::writeHeader(RequestHeader *header)
{
    // 1. Write SYNC pattern
    if (iface->is_spi())
    {
        // Short pattern for SPI
        SyncPattern sync = H2N_SYNC_PATTERN;
        iface->write(reinterpret_cast<uint8_t *>(&sync.short1),
                     SYNC_PATTERN_LEN);
    }
    else
    {
        // Long pattern for UART
        SyncPattern sync = H2N_SYNC_PATTERN;
        iface->write(reinterpret_cast<uint8_t *>(&sync.long1),
                     SYNC_PATTERN_LEN * 2);
    }

    // 2. Write body
    iface->write(reinterpret_cast<uint8_t *>(header), sizeof(RequestHeader));
}

void CC3135::dummyRead(size_t n)
{
    uint8_t dummy[n];
    iface->read(dummy, n);
}

}  // namespace Boardcore
