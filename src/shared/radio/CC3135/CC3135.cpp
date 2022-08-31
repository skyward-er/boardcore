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

#define TRY(expr)                                                      \
    do                                                                 \
    {                                                                  \
        Boardcore::CC3135::Error result = (expr);                      \
        if (result != Boardcore::CC3135::Error::NO_ERROR)              \
        {                                                              \
            TRACE("[cc3135] Error: %s @ %s:%d\n",                      \
                  Boardcore::CC3135::errorToStr(result), __FUNCTION__, \
                  __LINE__);                                           \
            return result;                                             \
        }                                                              \
    } while (0)

using namespace Boardcore::CC3135Defs;
using namespace miosix;

constexpr uint8_t TEST_CHANNEL = 64;

namespace Boardcore
{

CC3135::CC3135(std::unique_ptr<ICC3135Iface> &&iface)
    : iface(std::move(iface)), sd(128)
{
    irq_wait_thread = thread;
}

CC3135::~CC3135()
{
    socketClose(sd);
    sd = -1;
}

CC3135::Error CC3135::init(bool wait_for_init)
{
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

    // Setup transceiver mode
    // TRY(setupTransceiver());

    return Error::NO_ERROR;
}

CC3135::Error CC3135::prepareForReset()
{
    // We need the device in station mode
    TRY(wlanSetMode(ROLE_STA));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::setApChannel(uint8_t ch)
{
    TRY(wlanSet(WLAN_CFG_AP_ID, WLAN_AP_OPT_CHANNEL, Buffer::from(&ch)));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::getStatus(uint16_t mask, uint8_t &status)
{
    TRY(deviceGet(DEVICE_STATUS, mask, Buffer::from<uint8_t>(&status)));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::startRecv() { return socketRecv(sd, Buffer::null(), 0); }

CC3135::Error CC3135::dummyRecv() { return socketRecv(sd, Buffer::null(), 0); }

CC3135::Error CC3135::dummySend()
{
    struct Packet
    {
        uint32_t a = 0xdeadbabe;
        uint32_t b = 0xdeadbeef;
        uint32_t c = 0xbabebabe;
    };

    Packet packet;

    uint16_t flags =
        makeWlanRawRfTxParams(TEST_CHANNEL, RATE_1M, 0, PREAMBLE_LONG);
    TRY(socketSend2(sd, Buffer::from(&packet), flags));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::setupTransceiver()
{
    // TODO: This should really set the device in the correct mode and re-init

    // Reset connection policies
    TRY(wlanPolicySet(WLAN_POLICY_CONNECTION, 0, Buffer::null()));

    Error result = wlanDisconnect();

    // Check for not errors and continue
    if (result != Error::NO_ERROR && result != Error::WLAN_ALREADY_DISCONNECTED)
        return result;

    // Open transceiver socket
    // with respect of IEE802.11 medium access policies
    TRY(socketOpen(AF_RF, SOCK_RAW, TEST_CHANNEL, sd));
    TRACE("[cc3135] Socket opened: %d\n", sd);

    // As a sanity check, verify the socket type
    if (sdGetType(sd) != SdType::RAW_TRANSCEIVER)
        return Error::GENERIC_ERROR;

    return Error::NO_ERROR;
}

CC3135::Error CC3135::getVersion(DeviceVersion &version)
{
    version = {};
    return deviceGet(DEVICE_GENERAL, DEVICE_GENERAL_VERSION,
                     Buffer::from<DeviceVersion>(&version));
}

CC3135::Error CC3135::deviceGet(uint16_t set_id, uint16_t option, Buffer value)
{
    DeviceSetGet tx_command  = {};
    tx_command.device_set_id = set_id;
    tx_command.option        = option;
    tx_command.config_len    = value.len;

    DeviceSetGet rx_command = {};

    TRY(inoutPacketSync(OPCODE_DEVICE_DEVICEGET, Buffer::from(&tx_command),
                        Buffer::null(), OPCODE_DEVICE_DEVICEGETRESPONSE,
                        Buffer::from(&rx_command), value));

    return errorFromStatus(rx_command.status);
}

CC3135::Error CC3135::wlanSet(uint16_t config_id, uint16_t option, Buffer value)
{
    WlanCfgSetGet tx_command = {};
    tx_command.config_id     = config_id;
    tx_command.option        = option;
    tx_command.config_len    = value.len;

    BasicResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_WLAN_CFG_SET, Buffer::from(&tx_command), value,
                        OPCODE_WLAN_CFG_SET_RESPONSE, Buffer::from(&rx_command),
                        Buffer::null()));

    return errorFromStatus(rx_command.status);
}

CC3135::Error CC3135::wlanSetMode(CC3135Defs::Mode mode)
{
    WlanSetMode tx_command = {};
    tx_command.mode        = mode;

    BasicResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_WLAN_SET_MODE, Buffer::from(&tx_command),
                        Buffer::null(), OPCODE_WLAN_SET_MODE_RESPONSE,
                        Buffer::from(&rx_command), Buffer::null()));

    return errorFromStatus(rx_command.status);
}

CC3135::Error CC3135::wlanPolicySet(uint8_t type, uint8_t option, Buffer value)
{
    WlanPolicySetGet tx_command  = {};
    tx_command.policy_type       = type;
    tx_command.policy_option     = option;
    tx_command.policy_option_len = value.len;

    BasicResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_WLAN_POLICYSETCOMMAND, Buffer::from(&tx_command),
                        value, OPCODE_WLAN_POLICYSETRESPONSE,
                        Buffer::from(&rx_command), Buffer::null()));

    return errorFromStatus(rx_command.status);
}

CC3135::Error CC3135::wlanDisconnect()
{
    BasicResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_WLAN_WLANDISCONNECTCOMMAND, Buffer::null(),
                        Buffer::null(), OPCODE_WLAN_WLANDISCONNECTRESPONSE,
                        Buffer::from(&rx_command), Buffer::null()));

    return errorFromStatus(rx_command.status);
}

CC3135::Error CC3135::socketOpen(uint8_t domain, uint8_t type, uint8_t protocol,
                                 Sd &sd)
{
    SocketCommand tx_command = {};
    tx_command.domain        = domain;
    tx_command.type          = type;
    tx_command.protocol      = protocol;

    SocketResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_SOCKET_SOCKET, Buffer::from(&tx_command),
                        Buffer::null(), OPCODE_SOCKET_SOCKETRESPONSE,
                        Buffer::from(&rx_command), Buffer::null()));

    sd = rx_command.sd;
    return errorFromStatus(rx_command.status_or_len);
}

CC3135::Error CC3135::socketClose(Sd sd)
{
    CloseCommand tx_command = {};
    tx_command.sd           = sd;

    SocketResponse rx_command = {};

    TRY(inoutPacketSync(OPCODE_SOCKET_CLOSE, Buffer::from(&tx_command),
                        Buffer::null(), OPCODE_SOCKET_CLOSERESPONSE,
                        Buffer::from(&rx_command), Buffer::null()));

    return errorFromStatus(rx_command.status_or_len);
}

CC3135::Error CC3135::socketRecv(Sd sd, Buffer buffer, uint16_t flags)
{
    SendRecvCommand tx_command  = {};
    tx_command.sd               = sd;
    tx_command.status_or_len    = 300;
    tx_command.family_and_flags = flags & 0x0f;

    return writePacketSync(OPCODE_SOCKET_RECV, Buffer::from(&tx_command),
                           Buffer::null());
}

CC3135::Error CC3135::socketSend2(Sd sd, Buffer buffer, uint16_t flags)
{
    SendRecvCommand2 tx_command = {};
    tx_command.sd               = sd;
    tx_command.status_or_len    = buffer.len;
    tx_command.family_and_flags = flags & 0x0f;
    tx_command.flags            = flags;

    return writePacketSync(OPCODE_SOCKET_SEND, Buffer::from(&tx_command),
                           buffer);
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

void CC3135::run()
{
    // TODO: Implement a way to stop this thread
    while (true)
    {
        waitForIrq();
        // Thread::sleep(500);

        ResponseHeader header;
        {
            // Lock the device interface
            Lock<FastMutex> iface_lock(iface_mutex);

            Error result = readHeader(&header);

            if (result != Error::NO_ERROR)
            {
                TRACE("[cc3135] Error: %s\n", errorToStr(result));
                continue;
            }

            defaultPacketHandler(header);
        }
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

    size_t len = alignSize(header.inner.len);

    switch (header.inner.opcode)
    {
        case OPCODE_SOCKET_RECVASYNCRESPONSE:
        {
            SocketResponse rx_command = {};

            safeRead(len, Buffer::from(&rx_command));

            // if (rx_command.status_or_len < 0)
            //     noprintf("Status: %d\n", rx_command.status_or_len);

            // 8 byte proprietary header, format information?
            uint64_t header2;
            safeRead(len, Buffer::from(&header2));
            rx_command.status_or_len -= 8;

            // noprintf("Header: %8llx\n", header2);
            //
            // if (rx_command.status_or_len >= 0)
            // {
            //     noprintf("%d %d\n", len, rx_command.status_or_len);
            //
            //     uint8_t response[len];
            //     safeRead(len, Buffer{response, len});
            //
            //     for (int i = 0; i < rx_command.status_or_len; i++)
            //     {
            //         noprintf("%2x ", response[i]);
            //     }
            //     noprintf("\n");
            //
            //     while (1)
            //         ;
            // }

            break;
        }
        case OPCODE_DEVICE_DEVICE_ASYNC_GENERAL_ERROR:
        {
            BasicResponse rx_command = {};

            safeRead(len, Buffer::from(&rx_command));

            TRACE("[cc3135] Async general error occurred!\n");
            TRACE("- Status: %d\n", rx_command.status);
            TRACE("- Source: %d\n", rx_command.sender);

            break;
        }
        default:
            if (len > 0)
            {
                dummyRead(len);
                len = 0;
            }
    };

    // Dummy read rest of the data
    if (len > 0)
    {
        TRACE("[cc3135] %d bytes still in receive buffer!\n", len);
        dummyRead(len);
    }
}

CC3135::Error CC3135::inoutPacketSync(CC3135Defs::OpCode tx_opcode,
                                      CC3135::Buffer tx_command,
                                      CC3135::Buffer tx_payload,
                                      CC3135Defs::OpCode rx_opcode,
                                      CC3135::Buffer rx_command,
                                      CC3135::Buffer rx_payload)
{
    ServiceThreadLock service_thread_lock(this);

    // Lock the device interface
    Lock<FastMutex> iface_lock(iface_mutex);
    TRY(writePacket(tx_opcode, tx_command, tx_payload));
    TRY(readPacket(rx_opcode, rx_command, rx_payload));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::readPacketSync(CC3135Defs::OpCode opcode,
                                     CC3135::Buffer command,
                                     CC3135::Buffer payload)
{
    ServiceThreadLock service_thread_lock(this);

    // Lock the device interface
    Lock<FastMutex> iface_lock(iface_mutex);
    TRY(readPacket(opcode, command, payload));

    return Error::NO_ERROR;
}

CC3135::Error CC3135::writePacketSync(CC3135Defs::OpCode opcode,
                                      CC3135::Buffer command,
                                      CC3135::Buffer payload)
{
    // Lock the device interface
    Lock<FastMutex> iface_lock(iface_mutex);
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
            size_t len = alignSize(header.inner.len);

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
    size_t len = alignSize(command.len + payload.len);
    RequestHeader header{opcode, static_cast<uint16_t>(len)};

    TRY(writeHeader(&header));

    if (command.len > 0)
    {
        iface->write(command.ptr, command.len);
        len -= command.len;
    }

    if (payload.len > 0)
    {
        iface->write(payload.ptr, payload.len);
        len -= payload.len;
    }

    // Write final padding
    while (len > 0)
    {
        uint8_t buf[] = {0, 0, 0, 0};
        iface->write(buf, std::min(len, (size_t)4));
        len -= std::min(len, (size_t)4);
    }

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

    // for(int i = 0; i < 8; i++)
    //     noprintf("%x ", buf[i]);
    //
    // noprintf("\n");

    /*
    Here the TI driver does some weird stuff.
    Also the TI comment is wrong, soooo, I'll just skip that part
    */

    // 3. Scan for device SYNC
    while (true)
    {
        if ((getTick() - start) > TIMEOUT)
            return Error::SYNC_TIMEOUT;

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

            // for(int i = 4; i < 8; i++)
            //     noprintf("%x ", buf[i]);
            //
            // noprintf("\n");
        }
    }

    // 4. Scan for double syncs
    uint32_t sync;
    memcpy(&sync, &buf[0], 4);
    while (n2hSyncPatternMatch(sync, tx_seq_num))
    {
        if ((getTick() - start) > TIMEOUT)
            return Error::SYNC_TIMEOUT;

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
    uint8_t dummy[n];
    iface->read(dummy, n);
}

void CC3135::safeRead(size_t &len, Buffer buffer)
{
    if (buffer.len > 0 && len > 0)
    {
        iface->read(buffer.ptr, std::min(len, buffer.len));
        len -= std::min(len, buffer.len);
    }
}

CC3135::Error CC3135::errorFromStatus(int16_t status)
{
    if (status >= 0)
    {
        return Error::NO_ERROR;
    }

    TRACE("[cc3135] Non zero status: %d\n", status);
    switch (status)
    {
        case -2071:
            return Error::WLAN_ALREADY_DISCONNECTED;
        case -14343:
            return Error::DEVICE_LOCKED;
        default:
            return Error::GENERIC_ERROR;
    }
}

const char *CC3135::errorToStr(Error error)
{
    switch (error)
    {
        case Error::NO_ERROR:
            return "NO_ERROR";
        case Error::SYNC_TIMEOUT:
            return "SYNC_TIMEOUT";
        case Error::GENERIC_ERROR:
            return "GENERIC_ERROR";
        case Error::DEVICE_LOCKED:
            return "DEVICE_LOCKED";
        case Error::WLAN_ALREADY_DISCONNECTED:
            return "WLAN_ALREADY_DISCONNECTED";
        default:
            return "<unknown>";
    }
}

}  // namespace Boardcore
