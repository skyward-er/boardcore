/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo, Artem Glukhov, Alvise de Faveri Tron, Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Common.h>
#include <drivers/BusTemplate.h>
#include <drivers/Transceiver.h>
#include <interfaces-impl/hwmapping.h>
#include <algorithm>
#include <vector>

#include "ActiveObject.h"
#include "XbeeStatus.h"

using miosix::FastInterruptDisableLock;
using miosix::FastInterruptEnableLock;
using miosix::Thread;

using miosix::ConditionVariable;
using miosix::FastMutex;
using miosix::Lock;
using miosix::Unlock;

using std::vector;

namespace Xbee
{
// Constants

// How often should we check if we received a transmit status response.
static constexpr unsigned int SEND_STATUS_POLL_INTERVAL = 50;  // ms

static constexpr uint16_t MAX_PAYLOAD_LEN = 0xFFFF;

static constexpr uint8_t START_DELIMITER            = 0x7E;
static constexpr uint64_t BROADCAST_ADDR            = 0xFFFF;
static constexpr uint8_t ID_ACK_DISABLE             = 0x00;
static constexpr uint8_t MAX_BROADCAST_HOPS         = 0x00;
static constexpr uint8_t TX_STATUS_DELIVERY_SUCCESS = 0x00;

// NOTE: 0x43 = Point-Multipoint, No Ack, No route discovery. See transmit
// request frame bit options
static constexpr uint8_t TRANSMIT_OPTIONS = 0x43;

// Frame Sizes
static constexpr uint8_t API_HEADER_SIZE      = 3;
static constexpr uint8_t TX_FRAME_HEADER_SIZE = 14;
static constexpr uint8_t RX_FRAME_HEADER_SIZE = 12;
static constexpr uint8_t CHECKSUM_SIZE        = 1;

// Bit definitions for status frame
static constexpr uint8_t BIT_STATUS_RETRY_COUNT = 4;
static constexpr uint8_t BIT_STATUS_DELIVERY    = 5;
static constexpr uint8_t BIT_STATUS_DISCOVERY   = 6;

// Waiting thread to be woken
static miosix::Thread* waiting = nullptr;

/**
 * Handle ATTN interrupt, waking up the thread.
 */
static void __attribute__((used)) handleInterrupt()
{
    if (waiting)
    {
        // Wake
        waiting->IRQwakeup();

        if (waiting->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }

        waiting = nullptr;
    }
}

/**
 * WARNING: An IRQ linked with the ATTN pin of the Xbee module must be enabled
 * before using this class. See test/misc/xbee-bitrate for an example.
 */
template <typename Bus, class CS, class ATTN>
class Xbee : public Transceiver, public ActiveObject
{
public:
    Xbee(unsigned int send_timeout) : send_timeout(send_timeout) {}
    
    Xbee() : Xbee(5000) {}
    /*
     * Send a message through the XBee
     * Blocks until the message is sent (successfully or not)
     *
     * @param msg               Message to be sent
     * @param pkt_len           Lenght of the message
     * @return true             The message was sent correctly.
     * @return false            There was an error sending the message.
     */
    bool send(uint8_t* msg, size_t msg_len) override
    {
        if (msg == nullptr || msg_len == 0)
        {
            return false;
        }

        // Create a TX API packet
        vector<uint8_t> tx_pkt;
        buildTxPacket(tx_pkt, msg, msg_len);

        // Send the packet
        setTxBuf(tx_pkt.data(), tx_pkt.size());

        // Wake the runner thread in order to send the data
        wakeThread();

        // Wait until the XBee sends the message or a timeout expires
        unsigned int timeout = 0;
        while (!received_tx_status)
        {
            if (timeout > send_timeout)
            {
                // Timeout. Return error
                ++status.tx_timeout_count;
                return false;
            }
            Thread::sleep(SEND_STATUS_POLL_INTERVAL);
            timeout += SEND_STATUS_POLL_INTERVAL;
        }

        received_tx_status = false;
        if (status.tx_delivery_status != TX_STATUS_DELIVERY_SUCCESS)
        {
            printf("Error: %02X\n", status.tx_delivery_status);
        }
        return status.tx_delivery_status == TX_STATUS_DELIVERY_SUCCESS;
    }

    /*
     * Wait for a new message to be received by the XBee
     * @param rcv_buf Buffer to store the received message
     * @param rcv_buf_len Maximum length of the message to be received
     * @return true A message was successfully received
     * @return false Received invalid message
     */
    ssize_t receive(uint8_t* rcv_buf, size_t rcv_buf_len) override
    {
        Lock<FastMutex> l(rx_mutex);

        while (rx_buf.size() == 0)
        {
            rx_cond.wait(l);
        }

        if (rx_buf.size() > rcv_buf_len)
        {
            rx_buf.clear();

            return -1;
        }
        else
        {
            memcpy(rcv_buf, rx_buf.data(), rx_buf.size());

            rcv_buf_len = rx_buf.size();
            rx_buf.clear();

            return rcv_buf_len;
        }
    }

    void stop() override
    {
        should_stop = true;
        wakeThread();
        ActiveObject::stop();
    }

    XbeeStatus getStatus() { return status; }

protected:
    void run() override
    {
        while (!shouldStop())  // TODO: Stop condition
        {
            // Wait for RX or TX request
            {
                Lock<FastMutex> l(tx_mutex);
                miosix::FastInterruptDisableLock dLock;

                // Check if we have data to send or receive with interrupts
                // disabled to avoid race conditions
                if (ATTN::value() != 0 && tx_buf.size() == 0)
                {
                    // If ATTN is not asserted we wait until it is, or until we
                    // have something to send.
                    waiting = miosix::Thread::getCurrentThread();

                    while (waiting != 0)
                    {
                        waiting->IRQwait();
                        {
                            miosix::FastInterruptEnableLock eLock(dLock);
                            miosix::Unlock<FastMutex> ul(l);

                            miosix::Thread::yield();
                        }
                    }
                }
            }

            // Transfer any data on the tx buffer and receive any incoming data
            transferData();
        }
    }

private:
    void wakeThread()
    {
        FastInterruptDisableLock dLock;

        if (waiting)
        {
            waiting->IRQwakeup();
            waiting = nullptr;
        }
    }

    void transferData()
    {
        vector<uint8_t> buf;
        size_t tx_size = 0;

        {
            Lock<FastMutex> l(tx_mutex);

            // Check if we have data to send
            if (tx_buf.size() > 0)
            {
                // Copy the tx buffer in a local buffer and free it
                buf.insert(buf.end(), tx_buf.begin(), tx_buf.end());
                tx_size = tx_buf.size();

                tx_buf.clear();
            }
            else
            {
                // RX only
                // Reserve space for start delimiter & length field
                buf.resize(3, 0);
            }
        }

        ParserState state = ParserState::FIND_START;

        size_t index       = 0;
        size_t start_index = 0;
        size_t frame_len   = 0;
        uint8_t checksum   = 0;

        CS::low();

        vector<uint8_t> s;

        while (state != ParserState::END)
        {
            switch (state)
            {
                case ParserState::FIND_START:
                {
                    // buf.size() >= 3
                    if (index < buf.size())
                    {
                        Bus::transfer(buf.data() + index);
                        s.push_back(buf.at(index));
                        if (buf.at(index) == START_DELIMITER)
                        {
                            start_index = index;
                            state       = ParserState::READ_LENGTH;
                        }
                        ++index;
                    }
                    else  // If we're done transmiting or we didn't find a start
                          // delimiter
                    {
                        state = ParserState::END;
                    }
                    break;
                }
                case ParserState::READ_LENGTH:
                {
                    // Reserve space for length field
                    if (buf.size() < start_index + 3)
                    {
                        buf.resize(start_index + 3);
                    }

                    Bus::transfer(buf.data() + index, 2);
                    frame_len = (buf.at(index) << 8) + buf.at(index + 1);
                    index += 2;

                    state = ParserState::READ_FRAME;
                    break;
                }
                case ParserState::READ_FRAME:
                {
                    // Reserve for the rest of the packet
                    size_t required_size = index + frame_len + CHECKSUM_SIZE;
                    if (buf.size() < required_size)
                    {
                        buf.resize(required_size);
                    }

                    Bus::transfer(buf.data() + index, frame_len);
                    index += frame_len;
                    state = ParserState::READ_CHECKSUM;
                    break;
                }
                case ParserState::READ_CHECKSUM:
                {
                    Bus::transfer(buf.data() + index);
                    checksum = buf.at(index);
                    index++;

                    if (index >= tx_size)
                    {
                        // We sent all the data
                        state = ParserState::END;
                    }
                    else
                    {
                        // We still have data to send
                        state = ParserState::FIND_START;
                    }

                    // Handle the received packet
                    handleFrame(buf.data() + start_index + API_HEADER_SIZE,
                                frame_len, checksum);
                }
                case ParserState::END:
                {
                    break;
                }
            }
        }

        CS::high();
    }

    void handleFrame(uint8_t* frame_start, size_t frame_size, uint8_t checksum)
    {
        if (verifyChecksum(frame_start, frame_size, checksum))
        {
            // The first byte indicates the frame type
            switch (*frame_start)
            {
                case FRAMETYPE_RX_PACKET:
                {
                    uint8_t* payload_ptr = frame_start + RX_FRAME_HEADER_SIZE;
                    size_t payload_size  = frame_size - RX_FRAME_HEADER_SIZE;
                    if (payload_size > 0)
                    {
                        {
                            Lock<FastMutex> l(rx_mutex);

                            if (rx_buf.size() > 0)
                            {
                                rx_buf.clear();
                                ++status.rx_dropped_buffers;
                            }
                            rx_buf.insert(rx_buf.end(), payload_ptr,
                                          payload_ptr + payload_size);
                        }

                        rx_cond.signal();
                    }
                    break;
                }
                case FRAMETYPE_TRANSMIT_STATUS:
                {
                    status.tx_retry_count =
                        *(frame_start + BIT_STATUS_RETRY_COUNT);
                    status.tx_delivery_status =
                        *(frame_start + BIT_STATUS_DELIVERY);
                    status.tx_discovery_status =
                        *(frame_start + BIT_STATUS_DISCOVERY);

                    received_tx_status = true;
                    break;
                }
                default:
                    break;
            }
        }
        else
        {
            TRACE("[hf] Checksum failed\n");
            ++status.rx_wrong_checksum;
        }
    }

    void buildTxPacket(vector<uint8_t>& tx_pkt, uint8_t* msg, size_t msg_len)
    {
        tx_pkt.reserve(API_HEADER_SIZE + TX_FRAME_HEADER_SIZE + msg_len +
                       CHECKSUM_SIZE);

        tx_pkt.push_back(START_DELIMITER);

        uint16_t frame_len = msg_len + TX_FRAME_HEADER_SIZE;
        // invert pkt_len bytes order
        tx_pkt.push_back((frame_len & 0xff00) >> 8);
        tx_pkt.push_back(frame_len & 0xff);

        tx_pkt.push_back(FRAMETYPE_TX_REQUEST);

        // Any value != 0 indicates that we want a tx status response
        tx_pkt.push_back(0x01);

        // Split the address in bytes & add to the buffer
        uint8_t addr_buffer[8];
        memcpy(addr_buffer, &BROADCAST_ADDR, 8);

        for (int i = 7; i >= 0; i--)
        {
            tx_pkt.push_back(addr_buffer[i]);
        }

        // Reserved bytes
        tx_pkt.push_back(0xff);
        tx_pkt.push_back(0xfe);

        tx_pkt.push_back(MAX_BROADCAST_HOPS);

        tx_pkt.push_back(TRANSMIT_OPTIONS);

        // payload
        for (uint32_t i = 0; i < msg_len; i++)
        {
            tx_pkt.push_back(*(msg + i));
        }

        // Calculate & add checksum
        uint32_t checksum = 0;
        for (uint32_t i = 3; i < tx_pkt.size(); i++)
        {
            checksum += tx_pkt.at(i);
        }

        tx_pkt.push_back(0xff - (checksum & 0xff));
    }

    /**
     * Fill the tx buffer with the provided data
     */
    void setTxBuf(uint8_t* buf, size_t size)
    {
        Lock<FastMutex> l(tx_mutex);
        tx_buf.clear();
        tx_buf.insert(tx_buf.end(), buf, buf + size);
    }

    bool verifyChecksum(uint8_t* frame, size_t frame_len, uint8_t checksum)
    {
        // Sum all the bytes including checksum.
        // The sum can be stored in a uint8_t since we only care about the least
        // significant bits.
        uint8_t sum = checksum;
        for (size_t i = 0; i < frame_len; ++i)
        {
            sum += frame[i];
        }
        return sum == 0xFF;
    }

    enum class ParserState
    {
        FIND_START,
        READ_LENGTH,
        READ_FRAME,
        READ_CHECKSUM,
        END
    };

    enum FrameType : uint8_t
    {
        FRAMETYPE_AT_COMMAND                    = 0x08,
        FRAMETYPE_AT_QUEUE_PARAM_VALUE          = 0x09,
        FRAMETYPE_TX_REQUEST                    = 0x10,
        FRAMETYPE_EXPICIT_TX_REQUEST            = 0x11,
        FRAMETYPE_REMOTE_AT_REQUEST             = 0x17,
        FRAMETYPE_AT_COMMAND_RESPONSE           = 0x88,
        FRAMETYPE_MODEM_STATUS                  = 0x8A,
        FRAMETYPE_TRANSMIT_STATUS               = 0x8B,
        FRAMETYPE_ROUTE_INFO_PACKET             = 0x8D,
        FRAMETYPE_AGGREGATE_ADRESSING_UPDATE    = 0x8E,
        FRAMETYPE_RX_PACKET                     = 0x90,
        FRAMETYPE_EXPLICIT_RX_PACKET            = 0x91,
        FRAMETYPE_DATA_SAMPLE_RX_INDICATOR      = 0x92,
        FRAMETYPE_NODE_IDENTIFICATION_INDICATOR = 0x95,
        FRAMETYPE_REMOTE_COMMAND_RESPONSE       = 0x97
    };

    // How long to wait for a transmit status response.
    unsigned int send_timeout;

    FastMutex tx_mutex;
    vector<uint8_t> tx_buf;

    bool received_tx_status = false;

    FastMutex rx_mutex;
    vector<uint8_t> rx_buf;
    ConditionVariable rx_cond;

    XbeeStatus status;
};

}  // namespace Xbee