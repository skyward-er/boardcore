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

#pragma once

#include <arch/common/drivers/stm32_hardware_rng.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <radio/Xbee/APIFramesLog.h>
#include <radio/Xbee/Xbee.h>
#include <utils/Debug.h>
#include <utils/TestUtils/ThroughputCalculator.h>

#include <functional>

#include "ActiveObject.h"
#include "XbeeTestData.h"

using miosix::getTick;
using std::bind;
using std::function;

namespace Boardcore
{

static constexpr size_t RCV_BUF_SIZE       = 256;
static constexpr uint32_t PACKET_FIRST_INT = 0xF0E1C2B3;

/**
 * @brief Repeats the provided 32 bit value in the buffer, filling it.
 *
 * @param buf Buffer to fill
 * @param val Value to fill the buffer with
 * @param bufSize size of the buffer
 */
void memset32(uint8_t* buf, uint32_t val, int bufSize)
{
    int i = 0;
    while (i <= bufSize - (int)sizeof(val))
    {
        memcpy(buf + i, &val, sizeof(val));
        i += sizeof(val);
    }

    memcpy(buf + i, &val, bufSize % sizeof(val));
}

struct SendIntervalBase
{
    virtual unsigned int getInterval() const = 0;
};

struct ConstSendInterval : public SendIntervalBase
{
    explicit ConstSendInterval(unsigned int interval) : interval(interval) {}
    unsigned int interval;
    unsigned int getInterval() const override { return interval; }
};

struct RandSendInterval : public SendIntervalBase
{
    RandSendInterval(unsigned int min, unsigned int max)
        : min(min), diff(max - min)
    {
    }
    unsigned int min;
    unsigned int diff;
    unsigned int getInterval() const override
    {
        return (miosix::HardwareRng::instance().get() % diff) + min;
    }
};

class Sender : public ActiveObject
{
    friend class XbeeTransceiver;

public:
    Sender(Xbee::Xbee& xbee, Logger& logger, const SendIntervalBase& interval,
           size_t packetSize, long long expectedPacketInterval)
        : xbee(xbee), logger(logger), drCalc(expectedPacketInterval),
          interval(interval)
    {
        data.packetSize = packetSize;
        packetCounter   = 0;
    }

    TxData getTxData() const { return data; }

    DataRateResult getDataRate() { return drCalc.getResult(); }

protected:
    void run() override
    {
        long long loopStartTs = miosix::IRQgetTime() / 1e6;
        while (!shouldStop())
        {
            data.timeSinceLastSend = miosix::IRQgetTime() / 1e6 - loopStartTs;
            loopStartTs            = miosix::IRQgetTime() / 1e6;

            // Create packet
            memcpy(buf, &PACKET_FIRST_INT, sizeof(uint32_t));
            memset32(buf + sizeof(uint32_t), packetCounter++,
                     data.packetSize - sizeof(uint32_t));

            long long sendStartTs = miosix::IRQgetTime() / 1e6;

            bool result = xbee.send(buf, data.packetSize);

            data.timeToSend = miosix::IRQgetTime() / 1e6 - sendStartTs;
            data.timestamp  = sendStartTs;

            if (result)
            {
                ++data.txSuccessCounter;
                drCalc.addPacket(data.packetSize);
            }
            else
            {
                ++data.txFailCounter;
            }

            logger.log(data);

            miosix::Thread::sleepUntil(loopStartTs + interval.getInterval());
        }
    }

private:
    Xbee::Xbee& xbee;
    Logger& logger;
    ThroughputCalculator drCalc;
    TxData data;
    const SendIntervalBase& interval;

    uint32_t packetCounter = 0;

    uint8_t buf[Xbee::MAX_PACKET_PAYLOAD_LENGTH] = {0};
};

class Receiver : public ActiveObject
{
    friend class XbeeTransceiver;

public:
    Receiver(Xbee::Xbee& xbee, Logger& logger, long long expectedPacketInterval)
        : xbee(xbee), logger(logger), drCalc(expectedPacketInterval)
    {
    }

    void stop() override
    {
        stopFlag = true;
        xbee.wakeReceiver(true);
        ActiveObject::stop();
    }

    RxData getRxData() const { return data; }

    DataRateResult getDataRate() { return drCalc.getResult(); }

protected:
    void run() override
    {
        while (!shouldStop())
        {
            long long start = miosix::IRQgetTime() / 1e6;

            size_t len = xbee.receive(buf, RCV_BUF_SIZE);

            data.lastPacketTimestamp = miosix::IRQgetTime() / 1e6;
            data.timestamp           = start;

            ++data.rcvCount;

            if (len > sizeof(uint32_t))
            {
                data.pktSize = len;

                // Packet payload is a repeated sequential 32 bit number
                uint32_t num, strt;
                memcpy(&strt, buf, sizeof(uint32_t));
                memcpy(&num, buf + sizeof(uint32_t), sizeof(uint32_t));

                if (checkPacket(num, buf, len))
                {
                    drCalc.addPacket(len);
                    if (lastPacketNum > 0)
                    {
                        if (num > lastPacketNum)
                            data.packetsLost += num - lastPacketNum - 1;
                    }
                    lastPacketNum = num;
                }
                else
                {
                    TRACE(
                        "[XBeeTransceiver] Wrong packet payload! num: %u, "
                        "start: 0x%08X\n",
                        num, strt);
                    ++data.rcvWrongPayload;
                }
            }

            logger.log(data);
        }
    }

private:
    bool checkPacket(uint32_t expectedNum, uint8_t* packet, size_t packetSize)
    {
        if (memcmp(packet, &PACKET_FIRST_INT, sizeof(uint32_t)) == 0)
        {
            int i = sizeof(uint32_t);
            while (i <= (int)packetSize - (int)sizeof(expectedNum))
            {
                if (memcmp(packet + i, &expectedNum, sizeof(expectedNum)) != 0)
                {
                    return false;
                }
                i += sizeof(expectedNum);
            }
            memcpy(packet + i, &expectedNum, packetSize % sizeof(expectedNum));
            return true;
        }
        return false;
    }

    Xbee::Xbee& xbee;
    Logger& logger;
    ThroughputCalculator drCalc;
    RxData data{};

    uint8_t buf[RCV_BUF_SIZE] = {0};
    uint32_t lastPacketNum    = 0;
};

class XbeeTransceiver
{
public:
    XbeeTransceiver(Xbee::Xbee& xbee, Logger& logger,
                    const SendIntervalBase& sndInterval, size_t txPktSize,
                    long long expectedPacketInterval)
        // cppcheck-suppress noCopyConstructor
        // cppcheck-suppress noOperatorEq
        : sender(new Sender(xbee, logger, sndInterval, txPktSize,
                            expectedPacketInterval)),
          receiver(new Receiver(xbee, logger, expectedPacketInterval)),
          logger(logger)
    {
        xbee.setOnFrameReceivedListener(std::bind(
            &XbeeTransceiver::handleApiFrame, this, std::placeholders::_1));
    }

    ~XbeeTransceiver()
    {
        sender->stop();
        receiver->stop();

        delete sender;
        delete receiver;
    }

    void disableSender() { senderEnabled = false; }

    void disableReceiver() { receiverEnabled = false; }

    void start()
    {
        if (senderEnabled)
        {
            sender->start();
        }
        if (receiverEnabled)
        {
            receiver->start();
        }
    }

    void stop()
    {
        sender->stop();
        receiver->stop();
    }

    void setOnFrameReceivedListener(
        Xbee::Xbee::OnFrameReceivedListener frameListener)
    {
        this->frameListener = frameListener;
    }

    Sender& getSender() { return *sender; }

    Receiver& getReceiver() { return *receiver; }

private:
    void handleApiFrame(Xbee::APIFrame& frame)
    {
        logAPIFrame(frame);

        if (frameListener)
        {
            frameListener(frame);
        }

        if (frame.frameType == Xbee::FTYPE_AT_COMMAND_RESPONSE)
        {
            Xbee::ATCommandResponseFrame* at =
                frame.toFrameType<Xbee::ATCommandResponseFrame>();

            if (strncmp(at->getATCommand(), "DB", 2) == 0)
            {
                receiver->data.RSSI = -*at->getCommandDataPointer();
            }

            if (strncmp(at->getATCommand(), "ER", 2) == 0)
            {
                uint16_t errs;
                memcpy(&errs, at->getCommandDataPointer(), 2);
                receiver->data.rcvErrors = swapBytes16(errs);
            }
        }
    }

    void logAPIFrame(Xbee::APIFrame& frame)
    {
        using namespace Xbee;
        bool logged = false;
        switch (frame.frameType)
        {
            case FTYPE_AT_COMMAND:
            {
                ATCommandFrameLog dest;
                logged = ATCommandFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
            case FTYPE_AT_COMMAND_RESPONSE:
            {
                ATCommandResponseFrameLog dest;
                logged = ATCommandResponseFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
            case FTYPE_MODEM_STATUS:
            {
                ModemStatusFrameLog dest;
                logged = ModemStatusFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
            case FTYPE_TX_REQUEST:
            {
                TXRequestFrameLog dest;
                logged = TXRequestFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
            case FTYPE_TX_STATUS:
            {
                TXStatusFrameLog dest;
                logged = TXStatusFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
            case FTYPE_RX_PACKET_FRAME:
            {
                RXPacketFrameLog dest;
                logged = RXPacketFrameLog::toFrameType(frame, &dest);
                if (logged)
                {
                    logger.log(dest);
                }
                break;
            }
        }

        if (!logged)
        {
            APIFrameLog api;
            APIFrameLog::fromAPIFrame(frame, &api);
            logger.log(api);
        }
    }

    bool senderEnabled   = true;
    bool receiverEnabled = true;

    Sender* sender;
    Receiver* receiver;
    Logger& logger;

    Xbee::Xbee::OnFrameReceivedListener frameListener;
};

}  // namespace Boardcore
