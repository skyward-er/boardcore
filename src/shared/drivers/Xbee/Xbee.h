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
#include <algorithm>
#include <vector>

#include <miosix.h>
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
static constexpr uint8_t API_HEADER_SIZE = 3;

static constexpr uint8_t TX_FRAME_HEADER_SIZE    = 14;
static constexpr uint8_t RX_FRAME_HEADER_SIZE    = 12;
static constexpr uint8_t CHECKSUM_SIZE           = 1;
static constexpr uint8_t TX_STATUS_FRAME_SIZE    = 7;
static constexpr uint8_t MODEM_STATUS_FRAME_SIZE = 2;

// Bit definitions for status frame
static constexpr uint8_t BIT_STATUS_RETRY_COUNT = 4;
static constexpr uint8_t BIT_STATUS_DELIVERY    = 5;
static constexpr uint8_t BIT_STATUS_DISCOVERY   = 6;

// Waiting thread to be woken
// Defined extern so it can be accessed from multiple translation units
extern miosix::Thread* waiting;

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
template <typename Bus, class CS, class ATTN, class RST>
class Xbee : public Transceiver, public ActiveObject
{
public:
    Xbee(unsigned int send_timeout) : send_timeout(send_timeout)
    {
        reset();
        miosix::Thread::sleep(10);

        CS::low();
        miosix::Thread::sleep(1);
        CS::high();
        miosix::Thread::sleep(1);
    }

    Xbee() : Xbee(1000) {}
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
                TRACE("[Xbee] Send Timeout!\n");
                ++status.tx_timeout_count;
                return false;
            }
            Thread::sleep(SEND_STATUS_POLL_INTERVAL);
            timeout += SEND_STATUS_POLL_INTERVAL;
        }

        received_tx_status = false;
        if (status.tx_delivery_status != TX_STATUS_DELIVERY_SUCCESS)
        {
            TRACE("[Xbee] Error: %02X\n", status.tx_delivery_status);
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

        while (rx_frame.size() == 0)
        {
            rx_cond.wait(l);
        }

        if (rx_frame.size() > rcv_buf_len)
        {
            return -1;
        }
        else
        {
            memcpy(rcv_buf, rx_frame.data(), rx_frame.size());
            rcv_buf_len = rx_frame.size();

            rx_frame.clear();

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
    /**
     * Sends and receives data, puts the thread to sleep if nothing to
     * send/receive.
     */
    void run() override
    {
        while (!shouldStop())
        {
            // Wait for RX or TX request
            {
                miosix::FastInterruptDisableLock dLock;

                // Check if we have data to send (tx_buf.size > 0) or receive
                // (attn == 0) with disabled interrupts to avoid race conditions
                if (ATTN::value() != 0 && tx_buf.size() == 0)
                {
                    // If we have nothing to receive or send, wait.
                    waiting = miosix::Thread::getCurrentThread();

                    while (waiting != 0)
                    {
                        waiting->IRQwait();
                        {
                            miosix::FastInterruptEnableLock eLock(dLock);
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
    enum class ParseResult : uint8_t
    {
        IDLE,
        PARSING,
        SUCCESS,
        FAIL
    };

    enum class ParserState
    {
        FIND_START,
        READ_LENGTH_1,
        READ_LENGTH_2,
        READ_FRAME,
        READ_CHECKSUM
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

    void reset()
    {
        RST::mode(miosix::Mode::OPEN_DRAIN);
        RST::low();
        miosix::delayUs(500);
        RST::high();
    }

    /**
     * Wake the AO thread from another thread.
     * Cannot be called with disabled interrupts.
     */
    void wakeThread()
    {
        FastInterruptDisableLock dLock;

        if (waiting)
        {
            waiting->IRQwakeup();
            waiting = nullptr;
        }
    }

    /**
     * Transfer data to/from the Xbee.
     * Performs a full duplex transaction.
     */
    void transferData()
    {
        ParseResult result = ParseResult::IDLE;

        CS::low();
        vector<uint8_t> data;

        {
            // Make a local copy of the TX buffer
            FastInterruptDisableLock dLock;
            data = tx_buf;

            // Clear the tx buffer
            tx_buf.clear();
        }

        // If there is data to send
        if (data.size() > 0)
        {
            // Full duplex transfer, the data vector is replaced with received
            // data, if any.
            Bus::transfer(data.data(), data.size());

            // Parse the received data
            for (uint8_t rx : data)
            {
                result = parse(rx);

                // We received something
                if (result == ParseResult::SUCCESS)
                {
                    handleFrame(parser_buf);
                }
            }

            // If there's nothing more to parse, return
            if (result != ParseResult::PARSING)
            {
                CS::high();
                return;
            }

            // If there is more data to be received, continue below.
        }

        // Read until we have received a packet (or no packet is found)
        do
        {
            result = parse(Bus::read());
        } while (result == ParseResult::PARSING);

        if (result == ParseResult::SUCCESS)
        {
            handleFrame(parser_buf);
        }
        else if (result == ParseResult::FAIL)
        {
            TRACE("[Xbee] Read failed. Parser result: %d\n", (int)result);
        }

        CS::high();
    }

    /**
     * Parse received data one byte at a time, storing it in the parser_buf.
     * When a full packet is received, returns ParseResult::SUCCESS.
     * Returns ParseResult::FAIL if a full packet is received but checksum
     * verification fails.
     * In both cases, the frame is stored in parser_buf.
     *
     * Returns ParseResult::IDLE if no frame start delimiter has been found yet.
     * Returns ParseResult::PARSING if a start delimiter was found and a packet
     * is being parsed.
     *
     * @param byte byte to be parsed
     * @return
     */
    ParseResult parse(uint8_t byte)
    {
        switch (parser_state)
        {
            // Look for the start of frame delimiter
            case ParserState::FIND_START:
                if (byte == START_DELIMITER)
                {
                    parser_state = ParserState::READ_LENGTH_1;

                    // Frame start found, clear old rx buf
                    parser_buf.clear();
                    parser_packet_length = 0;
                }
                else
                {
                    return ParseResult::IDLE;
                }
                break;
            // Read most significant byte of the length
            case ParserState::READ_LENGTH_1:
                parser_packet_length = byte << 8;
                parser_state         = ParserState::READ_LENGTH_2;
                break;
            // Read least significant byte of the length
            case ParserState::READ_LENGTH_2:
                parser_packet_length += byte;

                parser_state = ParserState::READ_FRAME;

                // Now that we know how long the packet is, reserve memory to
                // store it
                parser_buf.reserve(parser_packet_length);
                break;
            // Read the data frame
            case ParserState::READ_FRAME:
                parser_buf.push_back(byte);

                if (parser_buf.size() == parser_packet_length)
                {
                    parser_state = ParserState::READ_CHECKSUM;
                }
                break;
            // Read & verify checksum
            case ParserState::READ_CHECKSUM:
                parser_state = ParserState::FIND_START;

                if (verifyChecksum(parser_buf, byte))
                {
                    return ParseResult::SUCCESS;
                }
                else
                {
                    ++status.rx_wrong_checksum;
                    TRACE("[Xbee] Rx checksum verification failed\n");
                    return ParseResult::FAIL;
                }
                break;
        }

        return ParseResult::PARSING;
    }

    void handleFrame(const vector<uint8_t>& frame)
    {
        if (frame.size() == 0)
        {
            return;
        }
        // The first byte indicates the frame type
        switch (frame[0])
        {
            case FRAMETYPE_RX_PACKET:
            {
                size_t payload_size = frame.size() - RX_FRAME_HEADER_SIZE;

                if (payload_size > 0)
                {
                    {
                        Lock<FastMutex> l(rx_mutex);
                        rx_frame.clear();
                        rx_frame.insert(rx_frame.end(),
                                        frame.begin() + RX_FRAME_HEADER_SIZE,
                                        frame.end());
                    }
                    rx_cond.signal();
                }
                break;
            }
            case FRAMETYPE_TRANSMIT_STATUS:
            {
                if (frame.size() == TX_STATUS_FRAME_SIZE)
                {
                    status.tx_retry_count      = frame[BIT_STATUS_RETRY_COUNT];
                    status.tx_delivery_status  = frame[BIT_STATUS_DELIVERY];
                    status.tx_discovery_status = frame[BIT_STATUS_DISCOVERY];

                    received_tx_status = true;
                }
                else
                {
                    TRACE("[Xbee] Wrong TX status frame size.\n");
                }
                break;
            }
            case FRAMETYPE_MODEM_STATUS:
            {
                if (frame.size() == MODEM_STATUS_FRAME_SIZE)
                {
                    TRACE("[Xbee] Modem status: %d\n", frame[1]);
                }
                else
                {
                    TRACE("[Xbee] Wrong MODEM status frame size.\n");
                }
                break;
            }
            default:
                break;
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
     * Set the tx buffer with the provided data.
     */
    void setTxBuf(uint8_t* buf, size_t size)
    {
        // We need to disable interrupts (instead of using a mutex) becouse we
        // need to synchronize the access to the tx buf in the waiting thread
        // before putting it to sleep (which requires disabling interrupts)
        FastInterruptDisableLock dLock;

        // Copy buf into tx_buf, removing old content
        tx_buf.clear();
        tx_buf.insert(tx_buf.end(), buf, buf + size);
    }

    bool verifyChecksum(vector<uint8_t> frame, uint8_t checksum)
    {
        // Sum all the bytes including checksum.
        // The sum can be stored in a uint8_t since we only care about the least
        // significant bits.
        uint8_t sum = checksum;
        for (size_t i = 0; i < frame.size(); ++i)
        {
            sum += frame[i];
        }
        return sum == 0xFF;
    }

    // How long to wait for a transmit status response.
    unsigned int send_timeout;

    vector<uint8_t> tx_buf;

    bool received_tx_status = false;

    FastMutex rx_mutex;
    vector<uint8_t> rx_frame;
    ConditionVariable rx_cond;

    ParserState parser_state    = ParserState::FIND_START;
    size_t parser_packet_length = 0;
    vector<uint8_t> parser_buf;

    XbeeStatus status;
};  // namespace Xbee

}  // namespace Xbee