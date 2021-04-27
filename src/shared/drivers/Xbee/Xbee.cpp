/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#include "Xbee.h"

#include <Debug.h>
#include <kernel/scheduler/scheduler.h>
#include <miosix.h>

#include <algorithm>

using miosix::FastMutex;
using miosix::Lock;
using miosix::Unlock;
using std::min;

namespace Xbee
{

Xbee::Xbee(SPIBusInterface& bus, GpioType cs, GpioType attn, GpioType rst,
           long long tx_timeout)
    : Xbee(bus, {}, cs, attn, rst, tx_timeout)
{
    spi_xbee.config.clock_div = SPIClockDivider::DIV128;
}

Xbee::Xbee(SPIBusInterface& bus, SPIBusConfig config, GpioType cs,
           GpioType attn, GpioType rst, long long tx_timeout)
    : spi_xbee(bus, cs, config), attn(attn), rst(rst), tx_timeout(tx_timeout)
{
    reset();
}

Xbee::~Xbee() { wakeReceiver(true); }

bool Xbee::send(uint8_t* pkt, size_t pkt_len)
{
    if (pkt_len > MAX_PACKET_PAYLOAD_LENGTH || pkt_len == 0)
    {
        TRACE("[Xbee] Invalid packet length (0< %u <= %u)\n", pkt_len,
              MAX_PACKET_PAYLOAD_LENGTH);
        return false;
    }
    long long start_tick = miosix::getTick();

    TXRequestFrame tx_req;
    uint8_t tx_frame_id = buildTXRequestFrame(tx_req, pkt, pkt_len);
    bool success        = false;
    bool status_timeout = true;
    {
        Lock<FastMutex> l(mutex_xbee_comm);
        writeFrame(tx_req);

        // Wait for a TX Status frame
        long long timeout_tick = miosix::getTick() + tx_timeout;

        while (waitForFrame(FTYPE_TX_STATUS, FRAME_POLL_INTERVAL, timeout_tick))
        {
            TXStatusFrame* f = parsing_api_frame.toFrameType<TXStatusFrame>();

            if (f->getFrameID() == tx_frame_id)
            {
                success        = f->getDeliveryStatus() == DELS_SUCCESS;
                status_timeout = false;
                break;
            }
            else
            {
                TRACE("[Xbee] Wrong tx_status ID\n");
            }
        }

        // Ak for total transmitted byte count for logging  purposes
        sendATCommandInternal("BC");
    }

    // If there is more data to receive, wake the receive() thread
    if (attn.value() == 0)
    {
        wakeReceiver();
    }
    if (status_timeout)
    {
        ++status.tx_timeout_count;
        TRACE("[Xbee] TX_STATUS timeout\n");
    }
    time_to_send_stats.add(miosix::getTick() - start_tick);
    return success;
}

ssize_t Xbee::receive(uint8_t* buf, size_t buf_max_size)
{
    while (true)
    {
        // We have data in the buffer pending to be returned
        if (!rx_frames_buf.isEmpty() || curr_rx_payload_ptr >= 0)
        {
            return fillReceiveBuf(buf, buf_max_size);
        }
        // No data in the buffer, but the xbee has data to return via SPI
        else if (attn.value() == 0)
        {
            Lock<FastMutex> l(mutex_xbee_comm);
            if (readRXFrame())
            {
                sendATCommandInternal("DB");  // Query last packet RSSI
                sendATCommandInternal("ER");  // Query receive error count
            }
        }
        // No data available at all: sleep until we have something to do
        else
        {
            {
                miosix::FastInterruptDisableLock dLock;
                receive_thread = miosix::Thread::getCurrentThread();

                while (receive_thread != 0)  // Avoid spurious wakeups
                {
                    receive_thread->IRQwait();
                    {
                        miosix::FastInterruptEnableLock eLock(dLock);
                        miosix::Thread::yield();
                    }
                }
            }

            // Forcefully return without receiving anything
            if (force_rcv_return)
            {
                force_rcv_return = false;
                return -1;
            }
        }
    }
}

XbeeStatus Xbee::getStatus()
{
    status.timestamp          = miosix::getTick();
    status.time_to_send_stats = time_to_send_stats.getStats();
    return status;
}

void Xbee::reset()
{
    Lock<FastMutex> l(mutex_xbee_comm);
    {
        miosix::FastInterruptDisableLock dLock();
        rst.mode(miosix::Mode::OPEN_DRAIN);
    }
    rst.low();
    miosix::delayUs(50);
    rst.high();

    // When the xbee is ready, we should assert SSEL to tell it to use
    // SPI, and it should provide us with a modem status frame
    long long timeout = miosix::getTick() + 1000;
    do
    {
        // Assert SSEL on every iteration as we don't exactly know when the
        // xbee will be ready.
        {
            SPIAcquireLock acq(spi_xbee);
            spi_xbee.cs.low();
            miosix::delayUs(10);
            spi_xbee.cs.high();
        }

        miosix::delayUs(50);

        if (attn.value() == 0 && readOneFrame() == ParseResult::SUCCESS)
        {
            handleFrame(parsing_api_frame);

            if (parsing_api_frame.frame_type == FTYPE_MODEM_STATUS)
            {
                break;
            }
        }
        miosix::Thread::sleep(5);
    } while (miosix::getTick() < timeout);
}

void Xbee::wakeReceiver(bool force_return)
{
    force_rcv_return = force_return;
    miosix::FastInterruptDisableLock dLock;

    if (receive_thread)
    {
        receive_thread->IRQwakeup();
        receive_thread = 0;
    }
}

void Xbee::handleATTNInterrupt()
{
    if (receive_thread)
    {
        receive_thread->IRQwakeup();
        if (receive_thread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }
        receive_thread = 0;
    }
}

size_t Xbee::fillReceiveBuf(uint8_t* buf, size_t buf_max_size)
{
    if (!rx_frames_buf.isEmpty() && curr_rx_payload_ptr == -1)
    {
        Lock<FastMutex> l(mutex_rx_frames);
        curr_rx_frame       = rx_frames_buf.pop();
        curr_rx_payload_ptr = 0;
    }

    if (curr_rx_payload_ptr >= 0)
    {
        size_t len_remaining_data =
            curr_rx_frame.getRXDataLength() - curr_rx_payload_ptr;

        // The buffer may be smaller than the data we need to return
        size_t len_to_copy = min(buf_max_size, len_remaining_data);

        memcpy(buf, curr_rx_frame.getRXDataPointer() + curr_rx_payload_ptr,
               len_to_copy);

        curr_rx_payload_ptr += len_to_copy;

        if (curr_rx_payload_ptr == curr_rx_frame.getRXDataLength())
        {
            // We've emptied the current frame
            curr_rx_payload_ptr = -1;
        }

        return len_to_copy;
    }
    

    return 0;
}

ParseResult Xbee::readOneFrame()
{
    SPIAcquireLock acq(spi_xbee);
    SPISelectLock sel(spi_xbee);

    ParseResult result = ParseResult::IDLE;
    do
    {
        result = parser.parse(spi_xbee.bus.read(), &parsing_api_frame);
    } while (attn.value() == 0 && result == ParseResult::PARSING);

    return result;
}

bool Xbee::readRXFrame()
{
    while (attn.value() == 0)
    {
        if (readOneFrame() == ParseResult::SUCCESS)
        {
            handleFrame(parsing_api_frame);

            if (parsing_api_frame.frame_type == FTYPE_RX_PACKET_FRAME)
            {
                return true;
            }
        }
    }
    return false;
}

void Xbee::writeFrame(APIFrame& frame)
{
    frame.timestamp = miosix::getTick();  // Only for logging purposes

    // Serialize the frame
    uint8_t tx_buf[MAX_API_FRAME_SIZE];
    size_t tx_buf_size = frame.toBytes(tx_buf);

    {
        SPITransaction spi(spi_xbee);
        spi.transfer(tx_buf, tx_buf_size);
    }

    // Pass the frame we just sent to the listener
    if (frame_listener)
    {
        frame_listener(frame);
    }

    // Full duplex spi transfer: we may have received valid data while we
    // were sending the frame.
    for (unsigned int i = 0; i < tx_buf_size; i++)
    {
        ParseResult res = parser.parse(tx_buf[i], &parsing_api_frame);
        if (res == ParseResult::SUCCESS)
        {
            handleFrame(parsing_api_frame);
        }
    }
}

bool Xbee::waitForFrame(uint8_t frame_type, unsigned int poll_interval,
                        long long timeout_tick)
{
    do
    {
        if (attn.value() == 0)
        {
            if (readOneFrame() == ParseResult::SUCCESS)
            {
                handleFrame(parsing_api_frame);

                if (parsing_api_frame.frame_type == frame_type)
                {
                    return true;
                }
            }
        }
        else
        {
            miosix::Thread::sleep(poll_interval);
        }
    } while (miosix::getTick() < timeout_tick);

    return false;
}

uint8_t Xbee::buildTXRequestFrame(TXRequestFrame& tx_req, uint8_t* pkt,
                                  size_t pkt_len)
{
    memcpy(tx_req.getRFDataPointer(), pkt, pkt_len);
    tx_req.setRFDataLength(pkt_len);

    uint8_t tx_frame_id = getNewFrameID();

    tx_req.setFrameID(tx_frame_id);

    tx_req.setDestAddress(ADDRESS_BROADCAST);
    tx_req.setBroadcastRadius(0);  // 0 = max hops, but we don't really care as
                                   // it does not apply to point-multipoint mode
    // Point-multipoint mode, disable ack, disable route discovery
    tx_req.setTransmitOptions(TO_DM_POINT_MULTIPOINT | TO_DISABLE_ACK |
                              TO_DISABLE_RD);
    tx_req.calcChecksum();

    return tx_frame_id;
}

void Xbee::sendATCommand(const char* cmd, uint8_t* params, size_t params_len)
{
    Lock<FastMutex> l(mutex_xbee_comm);

    sendATCommandInternal(0, cmd, params, params_len);
}

bool Xbee::sendATCommand(const char* cmd, ATCommandResponseFrame* response,
                         uint8_t* params, size_t params_len,
                         unsigned int timeout)
{
    Lock<FastMutex> l(mutex_xbee_comm);

    uint8_t tx_frame_id = sendATCommandInternal(cmd, params, params_len);

    bool success = false;

    long long timeout_tick = miosix::getTick() + timeout;

    while (waitForFrame(FTYPE_AT_COMMAND_RESPONSE, FRAME_POLL_INTERVAL,
                        timeout_tick))
    {
        ATCommandResponseFrame* f =
            parsing_api_frame.toFrameType<ATCommandResponseFrame>();

        if (f->getFrameID() == tx_frame_id &&
            strncmp(cmd, f->getATCommand(), 2) == 0)
        {
            memcpy(response, f, sizeof(ATCommandResponseFrame));
            success = true;
            break;
        }
    }

    return success;
}

uint8_t Xbee::sendATCommandInternal(const char* cmd, uint8_t* params,
                                    size_t params_len)
{
    return sendATCommandInternal(getNewFrameID(), cmd, params, params_len);
}

uint8_t Xbee::sendATCommandInternal(uint8_t tx_frame_id, const char* cmd,
                                    uint8_t* params, size_t params_len)
{
    // Build the AT command
    ATCommandFrame at;
    at.setATCommand(cmd);
    at.setFrameID(tx_frame_id);
    if (params_len > 0)
    {
        if (params_len > MAX_AT_COMMAND_PARAMS_LENGTH)
        {
            TRACE(
                "[Xbee] ERROR! AT Command payload too large. It was "
                "truncated.\n");
            params_len = MAX_AT_COMMAND_PARAMS_LENGTH;
        }
        at.setParameterSize(params_len);
        memcpy(at.getCommandDataPointer(), params, params_len);
    }

    at.calcChecksum();

    // Send it
    writeFrame(at);

    return tx_frame_id;
}

void Xbee::handleFrame(APIFrame& frame)
{
    // Set the timestamp to the frame
    frame.timestamp = miosix::getTick();

    switch (frame.frame_type)
    {
        case FTYPE_RX_PACKET_FRAME:
        {
            RXPacketFrame* pf = frame.toFrameType<RXPacketFrame>();
            {
                Lock<FastMutex> l(mutex_rx_frames);

                if (rx_frames_buf.isFull())
                {
                    ++status.rx_dropped_buffers;
                }
                rx_frames_buf.put(*pf);
                if (rx_frames_buf.count() > status.frame_buf_max_length)
                {
                    status.frame_buf_max_length = rx_frames_buf.count();
                }
            }
            wakeReceiver();
            break;
        }
        case FTYPE_MODEM_STATUS:
        {
            ModemStatusFrame* ms = frame.toFrameType<ModemStatusFrame>();

            switch (ms->getStatus())
            {
                case MS_HARDWARE_RESET:
                    TRACE("[Xbee] Modem status: Hardware reset\n");
                    break;
                case MS_WATCHDOG_TIMER_RESET:
                    TRACE("[Xbee] Modem status: Watchdog timer reset\n");
                    break;
            }
            break;
        }
        case FTYPE_TX_STATUS:
        {
            TXStatusFrame* ts     = frame.toFrameType<TXStatusFrame>();
            status.last_tx_status = ts->getDeliveryStatus();
            if (status.last_tx_status != DELS_SUCCESS)
            {
                status.last_tx_status_error = status.last_tx_status;
            }
            switch (status.last_tx_status)
            {
                case DELS_SUCCESS:
                    break;
                default:
                    TRACE(
                        "TX Status Error: %d (retries: %d, RD: %s)\n",
                        status.last_tx_status, ts->getTransmitRetryCount(),
                        ts->getDiscoveryStatus() == 2 ? "Enabled" : "Disabled");
                    break;
            }
            break;
        }
        default:
            break;
    }

    // Pass the frame to the listener
    if (frame_listener)
    {
        frame_listener(frame);
    }
}

void Xbee::setOnFrameReceivedListener(OnFrameReceivedListener listener)
{
    frame_listener = listener;
}

uint8_t Xbee::getNewFrameID()
{
    uint8_t tx_frame_id = frame_id_counter++;

    // Any value != 0 implies that we DO want a tx status response
    if (frame_id_counter == 0)
        frame_id_counter = 1;

    return tx_frame_id;
}

}  // namespace Xbee