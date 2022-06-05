/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include "Xbee.h"

#include <diagnostic/StackLogger.h>
#include <kernel/scheduler/scheduler.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <algorithm>

using miosix::FastMutex;
using miosix::Lock;
using miosix::Unlock;
using std::min;

namespace Boardcore
{

namespace Xbee
{

Xbee::Xbee(SPIBusInterface& bus, GpioType cs, GpioType attn, GpioType rst,
           long long txTimeout)
    : Xbee(bus, {}, cs, attn, rst, txTimeout)
{
    spiXbee.config.clockDivider = SPI::ClockDivider::DIV_128;
}

Xbee::Xbee(SPIBusInterface& bus, SPIBusConfig config, GpioType cs,
           GpioType attn, GpioType rst, long long txTimeout)
    : spiXbee(bus, cs, config), attn(attn), rst(rst), txTimeout(txTimeout)
{
    reset();
}

Xbee::~Xbee() { wakeReceiver(true); }

bool Xbee::send(uint8_t* pkt, size_t packetLength)
{
    if (packetLength > MAX_PACKET_PAYLOAD_LENGTH || packetLength == 0)
    {
        LOG_ERR(logger, "Invalid packet length (0< {} <= {})", packetLength,
                MAX_PACKET_PAYLOAD_LENGTH);
        return false;
    }
    long long startTick = miosix::getTick();

    TXRequestFrame txReq;
    uint8_t txFrameId  = buildTXRequestFrame(txReq, pkt, packetLength);
    bool success       = false;
    bool statusTimeout = true;
    {
        Lock<FastMutex> l(mutexXbeeCommunication);
        writeFrame(txReq);

        // Wait for a TX Status frame
        long long timeoutTick = miosix::getTick() + txTimeout;

        while (waitForFrame(FTYPE_TX_STATUS, FRAME_POLL_INTERVAL, timeoutTick))
        {
            TXStatusFrame* f = parsingApiFrame.toFrameType<TXStatusFrame>();

            if (f->getFrameID() == txFrameId)
            {
                success       = f->getDeliveryStatus() == DELS_SUCCESS;
                statusTimeout = false;
                break;
            }
            else
            {
                LOG_ERR(logger, "Wrong txStatus ID");
            }
        }

        // Ak for total transmitted byte count for logging  purposes
        // sendATCommandInternal("BC");
    }

    // If there is more data to receive, wake the receive() thread
    if (attn.value() == 0)
        wakeReceiver();

    if (statusTimeout)
    {
        ++status.txTimeoutCount;
        LOG_ERR(logger, "TX_STATUS timeout");
    }
    timeToSendStats.add(miosix::getTick() - startTick);

    StackLogger::getInstance().updateStack(THID_XBEE);

    return success;
}

ssize_t Xbee::receive(uint8_t* buf, size_t bufMaxSize)
{
    while (true)
    {
        // We have data in the buffer pending to be returned
        if (!rxFramesBuffer.isEmpty() || currRxPayloadPointer >= 0)
            return fillReceiveBuf(buf, bufMaxSize);

        // No data in the buffer, but the xbee has data to return via SPI
        else if (attn.value() == 0)
        {
            Lock<FastMutex> l(mutexXbeeCommunication);
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
                receiveThread = miosix::Thread::getCurrentThread();

                while (receiveThread != 0)  // Avoid spurious wakeups
                {
                    receiveThread->IRQwait();
                    {
                        miosix::FastInterruptEnableLock eLock(dLock);
                        miosix::Thread::yield();
                    }
                }
            }

            // Forcefully return without receiving anything
            if (forceRcvReturn)
            {
                forceRcvReturn = false;
                return -1;
            }
        }
    }
}

XbeeStatus Xbee::getStatus()
{
    status.timestamp       = miosix::getTick();
    status.timeToSendStats = timeToSendStats.getStats();
    return status;
}

void Xbee::reset()
{
    Lock<FastMutex> l(mutexXbeeCommunication);
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
            SPIAcquireLock acq(spiXbee);
            spiXbee.cs.low();
            miosix::delayUs(10);
            spiXbee.cs.high();
        }

        miosix::delayUs(50);

        if (attn.value() == 0 && readOneFrame() == ParseResult::SUCCESS)
        {
            handleFrame(parsingApiFrame);

            if (parsingApiFrame.frameType == FTYPE_MODEM_STATUS)
                break;
        }
        miosix::Thread::sleep(5);
    } while (miosix::getTick() < timeout);
}

void Xbee::wakeReceiver(bool forceReturn)
{
    forceRcvReturn = forceReturn;
    miosix::FastInterruptDisableLock dLock;

    if (receiveThread)
    {
        receiveThread->IRQwakeup();
        receiveThread = 0;
    }
}

void Xbee::handleATTNInterrupt()
{
    if (receiveThread)
    {
        receiveThread->IRQwakeup();
        if (receiveThread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
            miosix::Scheduler::IRQfindNextThread();

        receiveThread = 0;
    }
}

size_t Xbee::fillReceiveBuf(uint8_t* buf, size_t bufMaxSize)
{
    if (!rxFramesBuffer.isEmpty() && currRxPayloadPointer == -1)
    {
        Lock<FastMutex> l(mutexRxFrames);
        currRxFrame          = rxFramesBuffer.pop();
        currRxPayloadPointer = 0;
    }

    if (currRxPayloadPointer >= 0)
    {
        size_t lenRemainingData =
            currRxFrame.getRXDataLength() - currRxPayloadPointer;

        // The buffer may be smaller than the data we need to return
        size_t lenToCopy = min(bufMaxSize, lenRemainingData);

        memcpy(buf, currRxFrame.getRXDataPointer() + currRxPayloadPointer,
               lenToCopy);

        currRxPayloadPointer += lenToCopy;

        if (currRxPayloadPointer == currRxFrame.getRXDataLength())
        {
            // We've emptied the current frame
            currRxPayloadPointer = -1;
        }

        return lenToCopy;
    }

    return 0;
}

ParseResult Xbee::readOneFrame()
{
    SPIAcquireLock acq(spiXbee);
    SPISelectLock sel(spiXbee);

    ParseResult result = ParseResult::IDLE;
    do
    {
        result = parser.parse(spiXbee.bus.read(), &parsingApiFrame);
    } while (attn.value() == 0 && result == ParseResult::PARSING);

    return result;
}

bool Xbee::readRXFrame()
{
    while (attn.value() == 0)
    {
        if (readOneFrame() == ParseResult::SUCCESS)
        {
            handleFrame(parsingApiFrame);

            if (parsingApiFrame.frameType == FTYPE_RX_PACKET_FRAME)
                return true;
        }
    }
    return false;
}

void Xbee::writeFrame(APIFrame& frame)
{
    frame.timestamp = miosix::getTick();  // Only for logging purposes

    // Serialize the frame
    uint8_t txBuf[MAX_API_FRAME_SIZE];
    size_t txBufSize = frame.toBytes(txBuf);

    {
        SPITransaction spi(spiXbee);
        spi.transfer(txBuf, txBufSize);
    }

    // Pass the frame we just sent to the listener
    if (frameListener)
        frameListener(frame);

    // Full duplex spi transfer: we may have received valid data while we
    // were sending the frame.
    for (unsigned int i = 0; i < txBufSize; i++)
    {
        ParseResult res = parser.parse(txBuf[i], &parsingApiFrame);
        if (res == ParseResult::SUCCESS)
            handleFrame(parsingApiFrame);
    }
}

bool Xbee::waitForFrame(uint8_t frameType, unsigned int pollInterval,
                        long long timeoutTick)
{
    do
    {
        if (attn.value() == 0)
        {
            if (readOneFrame() == ParseResult::SUCCESS)
            {
                handleFrame(parsingApiFrame);

                if (parsingApiFrame.frameType == frameType)
                    return true;
            }
        }
        else
        {
            miosix::Thread::sleep(pollInterval);
        }
    } while (miosix::getTick() < timeoutTick);

    return false;
}

uint8_t Xbee::buildTXRequestFrame(TXRequestFrame& txReq, uint8_t* pkt,
                                  size_t packetLength)
{
    memcpy(txReq.getRFDataPointer(), pkt, packetLength);
    txReq.setRFDataLength(packetLength);

    uint8_t txFrameId = getNewFrameID();

    txReq.setFrameID(txFrameId);

    txReq.setDestAddress(ADDRESS_BROADCAST);
    txReq.setBroadcastRadius(0);  // 0 = max hops, but we don't really care as
                                  // it does not apply to point-multipoint mode
    // Point-multipoint mode, disable ack, disable route discovery
    txReq.setTransmitOptions(TO_DM_POINT_MULTIPOINT | TO_DISABLE_ACK |
                             TO_DISABLE_RD);
    txReq.calcChecksum();

    return txFrameId;
}

void Xbee::sendATCommand(const char* cmd, uint8_t* params, size_t paramsLen)
{
    Lock<FastMutex> l(mutexXbeeCommunication);

    sendATCommandInternal(0, cmd, params, paramsLen);
}

bool Xbee::sendATCommand(const char* cmd, ATCommandResponseFrame* response,
                         uint8_t* params, size_t paramsLen,
                         unsigned int timeout)
{
    Lock<FastMutex> l(mutexXbeeCommunication);

    uint8_t txFrameId = sendATCommandInternal(cmd, params, paramsLen);

    bool success = false;

    long long timeoutTick = miosix::getTick() + timeout;

    while (waitForFrame(FTYPE_AT_COMMAND_RESPONSE, FRAME_POLL_INTERVAL,
                        timeoutTick))
    {
        ATCommandResponseFrame* f =
            parsingApiFrame.toFrameType<ATCommandResponseFrame>();

        if (f->getFrameID() == txFrameId &&
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
                                    size_t paramsLen)
{
    return sendATCommandInternal(getNewFrameID(), cmd, params, paramsLen);
}

uint8_t Xbee::sendATCommandInternal(uint8_t txFrameId, const char* cmd,
                                    uint8_t* params, size_t paramsLen)
{
    // Build the AT command
    ATCommandFrame at;
    at.setATCommand(cmd);
    at.setFrameID(txFrameId);
    if (paramsLen > 0)
    {
        if (paramsLen > MAX_AT_COMMAND_PARAMS_LENGTH)
        {
            LOG_ERR(logger, "AT Command payload too large, it was truncated");
            paramsLen = MAX_AT_COMMAND_PARAMS_LENGTH;
        }
        at.setParameterSize(paramsLen);
        memcpy(at.getCommandDataPointer(), params, paramsLen);
    }

    at.calcChecksum();

    // Send it
    writeFrame(at);

    return txFrameId;
}

void Xbee::handleFrame(APIFrame& frame)
{
    // Set the timestamp to the frame
    frame.timestamp = miosix::getTick();

    switch (frame.frameType)
    {
        case FTYPE_RX_PACKET_FRAME:
        {
            RXPacketFrame* pf = frame.toFrameType<RXPacketFrame>();
            {
                Lock<FastMutex> l(mutexRxFrames);

                if (rxFramesBuffer.isFull())
                    ++status.rxDroppedBuffers;

                rxFramesBuffer.put(*pf);

                if (rxFramesBuffer.count() > status.frameBufMaxLength)
                    status.frameBufMaxLength = rxFramesBuffer.count();
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
                    LOG_DEBUG(logger, "Modem status: Hardware reset");
                    break;
                case MS_WATCHDOG_TIMER_RESET:
                    LOG_DEBUG(logger, "Modem status: Watchdog timer reset");
                    break;
            }
            break;
        }
        case FTYPE_TX_STATUS:
        {
            TXStatusFrame* ts   = frame.toFrameType<TXStatusFrame>();
            status.lastTxStatus = ts->getDeliveryStatus();

            if (status.lastTxStatus != DELS_SUCCESS)
                status.lastTxStatusError = status.lastTxStatus;

            switch (status.lastTxStatus)
            {
                case DELS_SUCCESS:
                    break;
                default:
                    LOG_ERR(
                        logger, "TX Status Error: {} (retries: {}, RD: {})",
                        status.lastTxStatus, ts->getTransmitRetryCount(),
                        ts->getDiscoveryStatus() == 2 ? "Enabled" : "Disabled");
                    break;
            }
            break;
        }
        default:
            break;
    }

    // Pass the frame to the listener
    if (frameListener)
        frameListener(frame);
}

void Xbee::setOnFrameReceivedListener(OnFrameReceivedListener listener)
{
    frameListener = listener;
}

uint8_t Xbee::getNewFrameID()
{
    uint8_t txFrameId = frameIdCounter++;

    // Any value != 0 implies that we DO want a tx status response
    if (frameIdCounter == 0)
        frameIdCounter = 1;

    return txFrameId;
}

}  // namespace Xbee

}  // namespace Boardcore
