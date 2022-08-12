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
    explicit CC3135(std::unique_ptr<ICC3135Iface> &&iface);

    void handleIrq();

    CC3135Defs::DeviceVersion getVersion();

    void setMode(CC3135Defs::Mode mode);

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

    //! Function for servicing async messages.
    void run() override;

    void defaultPacketHandler(CC3135Defs::ResponseHeader header);

    void devigeGet(uint8_t set_id, uint8_t option, Buffer result);

    // Functions dedicated to interrupt servicing

    //! Wait for an incoming interrupt (only callable in service thread).
    void waitForIrq();
    //! Install this thread as the service thread.
    void installAsServiceThread();
    //! Restore default service thread.
    void restoreDefaultServiceThread();

    // Functions for high level IO

    //! Write a packet in output and wait for a packet in input
    void inoutPacketSync(CC3135Defs::OpCode tx_opcode, Buffer tx_command,
                         Buffer tx_payload, CC3135Defs::OpCode rx_opcode,
                         Buffer rx_command, Buffer rx_payload);
    //! Read packet in input, with proper synchronization.
    void readPacketSync(CC3135Defs::OpCode opcode, Buffer command,
                        Buffer payload);
    //! Write a packet in output, with proper synchronization.
    void writePacketSync(CC3135Defs::OpCode opcode, Buffer command,
                         Buffer payload);

    // Functions for low level IO

    //! Read a single packet.
    void readPacket(CC3135Defs::OpCode opcode, Buffer command, Buffer payload);
    //! Write a single packet.
    void writePacket(CC3135Defs::OpCode opcode, Buffer command, Buffer payload);
    //! Read a packet header.
    void readHeader(CC3135Defs::ResponseHeader *header);
    //! Write a packet header.
    void writeHeader(CC3135Defs::RequestHeader *header);

    //! Read dummy n bytes.
    void dummyRead(size_t n);

    //! Safely read a buffer, does bound checking
    void safeRead(size_t &len, Buffer buffer);

    miosix::Thread *irq_wait_thread = nullptr;  //< Thread waiting on IRQ
    size_t irq_count                = 0;        //< Number of interrupts

    miosix::FastMutex iface_mutex;
    std::unique_ptr<ICC3135Iface> iface;

    uint8_t tx_seq_num = 0;
};

}  // namespace Boardcore
