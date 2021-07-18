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
#include <miosix.h>

#include <functional>

#include "ActiveObject.h"
#include "XbeeTestData.h"
#include "drivers/Xbee/APIFramesLog.h"
#include "drivers/Xbee/Xbee.h"
#include "logger/Logger.h"
#include "utils/testutils/ThroughputCalculator.h"

using miosix::getTick;
using std::bind;
using std::function;

static constexpr size_t RCV_BUF_SIZE       = 256;
static constexpr uint32_t PACKET_FIRST_INT = 0xF0E1C2B3;

/**
 * @brief Repeats the provided 32 bit value in the buffer, filling it.
 *
 * @param buf Buffer to fill
 * @param val Value to fill the buffer with
 * @param buf_size size of the buffer
 */
void memset32(uint8_t* buf, uint32_t val, int buf_size)
{
    int i = 0;
    while (i <= buf_size - (int)sizeof(val))
    {
        memcpy(buf + i, &val, sizeof(val));
        i += sizeof(val);
    }

    memcpy(buf + i, &val, buf_size % sizeof(val));
}

struct SendIntervalBase
{
    virtual unsigned int getInterval() const = 0;
};

struct ConstSendInterval : public SendIntervalBase
{
    ConstSendInterval(unsigned int interval) : interval(interval) {}
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
           size_t packet_size, long long expected_packet_interval)
        : xbee(xbee), logger(logger), dr_calc(expected_packet_interval),
          interval(interval)
    {
        data.packet_size = packet_size;
        packet_counter   = 0;
    }

    TxData getTxData() const { return data; }

    DataRateResult getDataRate() { return dr_calc.getResult(); }

protected:
    void run() override
    {
        long long loop_start_ts = getTick();
        while (!shouldStop())
        {
            data.time_since_last_send = getTick() - loop_start_ts;
            loop_start_ts             = getTick();

            // Create packet
            memcpy(buf, &PACKET_FIRST_INT, sizeof(uint32_t));
            memset32(buf + sizeof(uint32_t), packet_counter++,
                     data.packet_size - sizeof(uint32_t));

            long long send_start_ts = getTick();

            bool result = xbee.send(buf, data.packet_size);

            data.time_to_send = getTick() - send_start_ts;
            data.timestamp    = send_start_ts;

            if (result)
            {
                ++data.tx_success_counter;
                dr_calc.addPacket(data.packet_size);
            }
            else
            {
                ++data.tx_fail_counter;
            }

            logger.log(data);

            miosix::Thread::sleepUntil(loop_start_ts + interval.getInterval());
        }
    }

private:
    Xbee::Xbee& xbee;
    Logger& logger;
    ThroughputCalculator dr_calc;
    TxData data;
    const SendIntervalBase& interval;

    uint32_t packet_counter = 0;

    uint8_t buf[Xbee::MAX_PACKET_PAYLOAD_LENGTH];
};

class Receiver : public ActiveObject
{
    friend class XbeeTransceiver;

public:
    Receiver(Xbee::Xbee& xbee, Logger& logger,
             long long expected_packet_interval)
        : xbee(xbee), logger(logger), dr_calc(expected_packet_interval)
    {
    }

    void stop() override
    {
        should_stop = true;
        xbee.wakeReceiver(true);
        ActiveObject::stop();
    }

    RxData getRxData() const { return data; }

    DataRateResult getDataRate() { return dr_calc.getResult(); }

protected:
    void run() override
    {
        while (!shouldStop())
        {
            long long start = getTick();

            size_t len = xbee.receive(buf, RCV_BUF_SIZE);

            data.last_packet_timestamp = getTick();
            data.timestamp             = start;

            ++data.rcv_count;

            if (len > sizeof(uint32_t))
            {
                data.pkt_size = len;

                // Packet payload is a repeated sequential 32 bit number
                uint32_t num, strt;
                memcpy(&strt, buf, sizeof(uint32_t));
                memcpy(&num, buf + sizeof(uint32_t), sizeof(uint32_t));

                if (checkPacket(num, buf, len))
                {
                    dr_calc.addPacket(len);
                    if (last_packet_num > 0)
                    {
                        if (num > last_packet_num)
                            data.packets_lost += num - last_packet_num - 1;
                    }
                    last_packet_num = num;
                }
                else
                {
                    TRACE("[XBeeTransceiver] Wrong packet payload! num: %u, start: 0x%08X\n", num, strt);
                    ++data.rcv_wrong_payload;
                }
            }

            logger.log(data);
        }
    }

private:
    bool checkPacket(uint32_t expected_num, uint8_t* packet, size_t packet_size)
    {
        if (memcmp(packet, &PACKET_FIRST_INT, sizeof(uint32_t)) == 0)
        {
            int i = sizeof(uint32_t);
            while (i <= (int)packet_size - (int)sizeof(expected_num))
            {
                if (memcmp(packet + i, &expected_num, sizeof(expected_num)) !=
                    0)
                {
                    return false;
                }
                i += sizeof(expected_num);
            }
            memcpy(packet + i, &expected_num,
                   packet_size % sizeof(expected_num));
            return true;
        }
        return false;
    }

    Xbee::Xbee& xbee;
    Logger& logger;
    ThroughputCalculator dr_calc;
    RxData data{};

    uint8_t buf[RCV_BUF_SIZE];
    uint32_t last_packet_num = 0;
};

class XbeeTransceiver
{
public:
    XbeeTransceiver(Xbee::Xbee& xbee, Logger& logger,
                    const SendIntervalBase& snd_interval, size_t tx_pkt_size,
                    long long expected_packet_interval)
        : sender(new Sender(xbee, logger, snd_interval, tx_pkt_size,
                            expected_packet_interval)),
          receiver(new Receiver(xbee, logger, expected_packet_interval)),
          logger(logger)
    {
        using namespace std::placeholders;
        xbee.setOnFrameReceivedListener(
            bind(&XbeeTransceiver::handleApiFrame, this, _1));
    }

    ~XbeeTransceiver()
    {
        sender->stop();
        receiver->stop();

        delete sender;
        delete receiver;
    }

    void disableSender() { sender_enabled = false; }

    void disableReceiver() { receiver_enabled = false; }

    void start()
    {
        if (sender_enabled)
        {
            sender->start();
        }
        if (receiver_enabled)
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
        Xbee::Xbee::OnFrameReceivedListener frame_listener)
    {
        this->frame_listener = frame_listener;
    }

    Sender& getSender() { return *sender; }

    Receiver& getReceiver() { return *receiver; }

private:
    void handleApiFrame(Xbee::APIFrame& frame)
    {
        logAPIFrame(frame);

        if (frame_listener)
        {
            frame_listener(frame);
        }

        if (frame.frame_type == Xbee::FTYPE_AT_COMMAND_RESPONSE)
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
                receiver->data.rcv_errors = swapBytes16(errs);
            }
        }
    }

    void logAPIFrame(Xbee::APIFrame& frame)
    {
        using namespace Xbee;
        bool logged = false;
        switch (frame.frame_type)
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

    bool sender_enabled   = true;
    bool receiver_enabled = true;

    Sender* sender;
    Receiver* receiver;
    Logger& logger;

    Xbee::Xbee::OnFrameReceivedListener frame_listener;
};