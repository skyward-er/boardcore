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

#define TRY(expr)                                            \
    do                                                       \
    {                                                        \
        Boardcore::CC3135::Error result = (expr);            \
        if (result != Boardcore::CC3135::Error::NO_ERROR)    \
        {                                                    \
            TRACE("[cc3135] NWP error@ %s\n", __FUNCTION__); \
            return result;                                   \
        }                                                    \
    } while (0)

using namespace Boardcore::CC3135Defs;
using namespace miosix;

namespace Boardcore
{

CC3135::CC3135(std::unique_ptr<ICC3135Iface> &&iface) : iface(std::move(iface))
{
    irq_wait_thread = thread;
}

CC3135::Error CC3135::init(bool wait_for_init)
{
    if (iface->is_spi())
        dummyDeviceRead();

    if (wait_for_init)
    {
        DeviceInitInfo init_info = {};
        TRY(readPacketSync(OPCODE_DEVICE_INITCOMPLETE, Buffer::from(&init_info),
                           Buffer::null()));

        TRACE(
            "[cc3135] Init completed:\n"
            "- Status: %8x\n"
            "- Chip Id: %lx\n"
            "- More Data: %8x\n",
            init_info.status, init_info.chip_id, init_info.more_data);
    }

    // Start internal thread
    this->start();

    return Error::NO_ERROR;
}

CC3135::Error CC3135::getVersion(DeviceVersion &version)
{
    version = {};
    return devigeGet(1, 12, Buffer::from<DeviceVersion>(&version));
}

CC3135::Error CC3135::devigeGet(uint8_t set_id, uint8_t option, Buffer result)
{
    DeviceSetGet tx_command  = {};
    tx_command.device_set_id = set_id;
    tx_command.option        = option;

    DeviceSetGet rx_command = {};

    return inoutPacketSync(OPCODE_DEVICE_DEVICEGET, Buffer::from(&tx_command),
                           Buffer::null(), OPCODE_DEVICE_DEVICEGETRESPONSE,
                           Buffer::from(&rx_command), result);
}

CC3135::Error CC3135::dummyDeviceRead()
{
    ResponseHeader header;
    TRY(readHeader(&header));

    defaultPacketHandler(header);

    return Error::NO_ERROR;
}

CC3135::Error CC3135::setMode(CC3135Defs::Mode mode)
{
    WlanSetMode tx_command = {};
    tx_command.mode        = mode;

    BasicResponse rx_command = {};

    return inoutPacketSync(OPCODE_WLAN_SET_MODE, Buffer::from(&tx_command),
                           Buffer::null(), OPCODE_WLAN_SET_MODE_RESPONSE,
                           Buffer::from(&rx_command), Buffer::null());
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

    size_t len = header.inner.len;

    // Dummy read rest of the data
    if (len > 0)
        dummyRead(len);
}

void CC3135::run()
{
    // TODO: Implement a way to stop this thread
    while (true)
    {
        waitForIrq();
        // Thread::sleep(500);

        {
            // Lock the device interface
            Lock<FastMutex> lock(iface_mutex);

            ResponseHeader header;
            Error result = readHeader(&header);

            if (result != Error::NO_ERROR)
            {
                TRACE("[cc3135] NWP error!\n");
            }
            else
            {
                defaultPacketHandler(header);
            }
        }
    }
}

CC3135::Error CC3135::inoutPacketSync(CC3135Defs::OpCode tx_opcode,
                                      CC3135::Buffer tx_command,
                                      CC3135::Buffer tx_payload,
                                      CC3135Defs::OpCode rx_opcode,
                                      CC3135::Buffer rx_command,
                                      CC3135::Buffer rx_payload)
{
    installAsServiceThread();

    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    TRY(writePacket(tx_opcode, tx_command, tx_payload));

    TRY(readPacket(rx_opcode, rx_command, rx_payload));

    restoreDefaultServiceThread();

    return Error::NO_ERROR;
}

CC3135::Error CC3135::readPacketSync(CC3135Defs::OpCode opcode,
                                     CC3135::Buffer command,
                                     CC3135::Buffer payload)
{
    installAsServiceThread();

    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    TRY(readPacket(opcode, command, payload));

    restoreDefaultServiceThread();

    return Error::NO_ERROR;
}

CC3135::Error CC3135::writePacketSync(CC3135Defs::OpCode opcode,
                                      CC3135::Buffer command,
                                      CC3135::Buffer payload)
{
    // Lock the device interface
    Lock<FastMutex> lock(iface_mutex);
    TRY(writePacket(opcode, command, payload));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::readPacket(OpCode opcode, CC3135::Buffer command,
                                 CC3135::Buffer payload)
{
    while (true)
    {
        waitForIrq();
        // Thread::sleep(500);

        // Locking the interface is not needed

        ResponseHeader header = {};
        TRY(readHeader(&header));

        if (header.inner.opcode != opcode)
        {
            defaultPacketHandler(header);
        }
        else
        {
            // Read the rest of the packet
            size_t len = header.inner.len;

            safeRead(len, command);
            safeRead(len, payload);

            // Read tail of remaining data
            if (len > 0)
                dummyRead(len);

            break;
        }
    }

    return Error::NO_ERROR;
}

CC3135::Error CC3135::writePacket(OpCode opcode, CC3135::Buffer command,
                                  CC3135::Buffer payload)
{
    RequestHeader header{opcode,
                         static_cast<uint16_t>(command.len + payload.len)};

    TRY(writeHeader(&header));

    if (command.len > 0)
        iface->write(command.ptr, command.len);

    if (payload.len > 0)
        iface->write(payload.ptr, payload.len);

    return Error::NO_ERROR;
}

CC3135::Error CC3135::readHeader(ResponseHeader *header)
{
    // 20 kernel ticks, or 100ms
    const int TIMEOUT = 2000;

    long long start = getTick();

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
        if ((getTick() - start) > TIMEOUT)
            return Error::NWP_TIMEOUT;

        // Reads in SPI are always more than 4 bytes
        bool found = false;
        int i;
        for (i = 0; i < 4; i++)
        {
            // Try and find the SYNC here
            uint32_t sync;
            memcpy(&sync, &buf[i], 4);

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
    uint32_t sync;
    memcpy(&sync, &buf[0], 4);
    while (n2hSyncPatternMatch(sync, tx_seq_num))
    {
        if ((getTick() - start) > TIMEOUT)
            return Error::NWP_TIMEOUT;

        memmove(&buf[0], &buf[4], 4);
        iface->read(&buf[4], 4);

        memcpy(&sync, &buf[0], 4);
    }

    tx_seq_num++;

    // 5. Read rest of the header
    iface->read(&buf[8], sizeof(ResponseHeader) - 8);
    memcpy(header, &buf[0], sizeof(ResponseHeader));

    // 6. Adjust for bigger response header
    header->inner.len -= sizeof(ResponseHeader) - sizeof(GenericHeader);

    return Error::NO_ERROR;
}

CC3135::Error CC3135::writeHeader(RequestHeader *header)
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

    return Error::NO_ERROR;
}

void CC3135::dummyRead(size_t n)
{
    // Avoid HUGE stack allocations
    uint8_t dummy[256];

    while (n > 0)
    {
        iface->read(dummy, std::min(n, sizeof(dummy)));
        n -= std::min(n, sizeof(dummy));
    }
}

void CC3135::safeRead(size_t &len, Buffer buffer)
{
    if (buffer.len > 0 && len > 0)
    {
        iface->read(buffer.ptr, std::min(len, buffer.len));
        len -= std::min(len, buffer.len);
    }
}

}  // namespace Boardcore
