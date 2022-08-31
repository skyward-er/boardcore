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

#include <ActiveObject.h>
#include <miosix.h>

#include <functional>
#include <memory>

#include "CC3135Defs.h"
#include "CC3135Iface.h"

/*
TODO(davide.mor): Write a small description of the CC3135

TODO(davide.mor): Write about scatter/gather IO
*/

namespace Boardcore
{

class CC3135 : ActiveObject
{
public:
    constexpr static size_t MTU = 1536;

    enum class Error
    {
        NO_ERROR,                   //< No error occured.
        SYNC_TIMEOUT,               //< The NWP did not respond.
        GENERIC_ERROR,              //< Generic error class.
        WLAN_ALREADY_DISCONNECTED,  //< Wlan is already disconnected.
        DEVICE_LOCKED,              //< Device is locked
    };

    explicit CC3135(std::unique_ptr<ICC3135Iface> &&iface);
    ~CC3135();

    CC3135::Error init(bool wait_for_init);

    void handleIrq();

    CC3135::Error getVersion(CC3135Defs::DeviceVersion &version);

    CC3135::Error startRecv();

    CC3135::Error dummyRecv();
    CC3135::Error dummySend();

    CC3135::Error prepareForReset();
    CC3135::Error setApChannel(uint8_t ch);

    CC3135::Error getStatus(uint16_t mask, uint8_t &status);

    static const char *errorToStr(Error error);

private:
    //! Simple buffer for scatter/gather IO
    struct Buffer
    {
        uint8_t *ptr;
        size_t len;

        template <typename T>
        static Buffer from(T *data)
        {
            return {reinterpret_cast<uint8_t *>(data), sizeof(T)};
        }

        static Buffer null() { return {nullptr, 0}; }
    };

    class ServiceThreadLock
    {
    public:
        explicit ServiceThreadLock(CC3135 *parent) : parent(parent)
        {
            parent->installAsServiceThread();
        }

        ~ServiceThreadLock() { parent->restoreDefaultServiceThread(); }

    private:
        CC3135 *parent;
    };

    CC3135::Error setupTransceiver();

    //! Function for servicing async messages.
    void run() override;

    void defaultPacketHandler(CC3135Defs::ResponseHeader header);

    // Part of the device API

    CC3135::Error deviceGet(uint16_t set_id, uint16_t option, Buffer value);

    CC3135::Error wlanSet(uint16_t config_id, uint16_t option, Buffer value);

    CC3135::Error wlanSetMode(CC3135Defs::Mode mode);

    CC3135::Error wlanPolicySet(uint8_t type, uint8_t option, Buffer value);

    CC3135::Error wlanDisconnect();

    CC3135::Error socketOpen(uint8_t domain, uint8_t type, uint8_t protocol,
                             CC3135Defs::Sd &sd);

    CC3135::Error socketClose(CC3135Defs::Sd sd);

    CC3135::Error socketRecv(CC3135Defs::Sd sd, Buffer buffer, uint16_t flags);

    // CC3135::Error socketSend(CC3135Defs::Sd sd, Buffer buffer, uint8_t
    // flags);

    CC3135::Error socketSend2(CC3135Defs::Sd sd, Buffer buffer, uint16_t flags);

    // Functions dedicated to interrupt servicing

    //! Wait for an incoming interrupt (only callable in service thread).
    void waitForIrq();
    //! Install this thread as the service thread.
    void installAsServiceThread();
    //! Restore default service thread.
    void restoreDefaultServiceThread();

    // Functions for high level IO

    //! Write a packet in output and wait for a packet in input
    CC3135::Error inoutPacketSync(CC3135Defs::OpCode tx_opcode,
                                  Buffer tx_command, Buffer tx_payload,
                                  CC3135Defs::OpCode rx_opcode,
                                  Buffer rx_command, Buffer rx_payload);
    //! Read packet in input, with proper synchronization.
    CC3135::Error readPacketSync(CC3135Defs::OpCode opcode, Buffer command,
                                 Buffer payload);
    //! Write a packet in output, with proper synchronization.
    CC3135::Error writePacketSync(CC3135Defs::OpCode opcode, Buffer command,
                                  Buffer payload);

    // Functions for low level IO

    //! Read a single packet.
    CC3135::Error readPacket(CC3135Defs::OpCode opcode, Buffer command,
                             Buffer payload);
    //! Write a single packet.
    CC3135::Error writePacket(CC3135Defs::OpCode opcode, Buffer command,
                              Buffer payload);
    //! Read a packet header.
    CC3135::Error readHeader(CC3135Defs::ResponseHeader *header);
    //! Write a packet header.
    CC3135::Error writeHeader(CC3135Defs::RequestHeader *header);

    //! Read dummy n bytes.
    void dummyRead(size_t n);

    //! Safely read a buffer, does bound checking
    void safeRead(size_t &len, Buffer buffer);

    static CC3135::Error errorFromStatus(int16_t status);

    miosix::Thread *irq_wait_thread = nullptr;  //< Thread waiting on IRQ
    size_t irq_count                = 0;        //< Number of interrupts

    miosix::FastMutex iface_mutex;
    std::unique_ptr<ICC3135Iface> iface;

    uint8_t tx_seq_num = 0;

    CC3135Defs::Sd sd;  //< Socket descriptor
};

}  // namespace Boardcore
