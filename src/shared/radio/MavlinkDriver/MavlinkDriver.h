/* Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <vector>

/**
 * This object includes only the protocol header (`protocol.h`). To use this
 * driver, you should include YOUR OWN implementation of the messages definition
 * (`mavlink.h`) before including this header.
 */
#ifndef MAVLINK_H
#error \
    "[MavlinkDriver] Mavlink header not found! Please include your mavlink.h \
implementation before including MavlinkDriver.h"
#endif

#include <diagnostic/PrintLogger.h>
#include <diagnostic/SkywardStack.h>
#include <diagnostic/StackLogger.h>
#include <mavlink_lib/mavlink_types.h>
#include <radio/Transceiver.h>
#include <utils/collections/SyncPacketQueue.h>

#include "MavlinkStatus.h"

namespace Boardcore
{

/**
 * @brief The MavlinkDriver object offers an interface to send and receive from
 * a Transceiver object using an implementation of the Mavlink protocol.
 *
 * See `src/tests/drivers/test-mavlink.cpp` for an example.
 *
 * @tparam PktLength Maximum length in bytes of each transceiver packet.
 * @tparam OutQueueSize Max number of transceiver packets in the output queue.
 * @tparam MavMsgLength Max length of a mavlink message. By default is 255 the
 * maximum possible but can be replaces with MAVLINK_MAX_DIALECT_PAYLOAD_SIZE
 * for a specific protocol.
 */
template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength = MAVLINK_MAX_PAYLOAD_LEN>
class MavlinkDriver
{

public:
    /**
     * @brief Initializes all data structures.
     *
     * @param device Transceiver used to send and receive messages.
     * after this time the message will be automatically sent [ms].
     */
    MavlinkDriver(Transceiver* device) : device(device)
    {
        memset(&status, 0, sizeof(MavlinkStatus));
    };

    /**
     * @brief Tells whether the driver was started.
     */
    bool isStarted() { return sndStarted && rcvStarted; };

    /**
     * @brief Stops sender and receiver threads.
     */
    void stop()
    {
        stopFlag = true;
        sndThread->join();  // Wait for sender to stop
    };

    /**
     * @brief Synchronized status getter.
     */
    MavlinkStatus getStatus()
    {
        miosix::Lock<miosix::FastMutex> l(mtxStatus);
        status.timestamp = TimestampTimer::getTimestamp();
        return status;
    };

    /**
     * @brief Non-blocking send function, puts the message in a queue.
     * Message is discarded if the queue is full.
     *
     * @param msg Message to send (mavlink struct).
     * @return True if the message could be enqueued.
     */
    bool enqueueMsg(const mavlink_message_t& msg)
    {
        // Convert mavlink message to a byte array
        uint8_t msgTempBuf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MavMsgLength];
        int msgLen = mavlink_msg_to_send_buffer(msgTempBuf, &msg);
        // Append the message to the queue
        bool appended = outQueue.put(msgTempBuf, msgLen);
        // Update stats
        updateQueueStats(appended);
        // Return ok even if a packet was discarded
        return appended;
    };

    /**
     * @brief Enqueue a raw packet message into the sync packet queue.
     *
     * @param msg Message to send.
     * @param size Length in bytes.
     * @return True if the message was enqueued.
     */
    bool enqueueRaw(uint8_t* msg, size_t size)
    {
        // Append message to the queue
        bool appended = outQueue.put(msg, size);
        // Update stats
        updateQueueStats(appended);
        // Return ok even if a packet was discarded
        return appended;
    };

    /**
     * @brief  Start the receiving and sending threads.
     *
     * @return False if at least one could not start.
     */
    bool start()
    {
        stopFlag = false;

        // Start sender (joinable thread)
        if (!sndStarted)
        {
            sndThread = miosix::Thread::create(
                [](void* arg)
                { reinterpret_cast<MavlinkDriver*>(arg)->runSender(); },
                skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
                reinterpret_cast<void*>(this), miosix::Thread::JOINABLE);

            if (sndThread != nullptr)
                sndStarted = true;
            else
                LOG_ERR(logger, "Could not start sender!");
        }

        // Start receiver
        if (!rcvStarted)
        {
            rcvThread = miosix::Thread::create(
                [](void* arg)
                { reinterpret_cast<MavlinkDriver*>(arg)->runReceiver(); },
                skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
                reinterpret_cast<void*>(this));

            if (rcvThread != nullptr)
                rcvStarted = true;
            else
                LOG_ERR(logger, "Could not start receiver!");
        }

        if (sndStarted && rcvStarted)
            LOG_DEBUG(logger, "Sender and receiver started");

        return sndStarted && rcvStarted;
    };

protected:
    /**
     * @brief Receiver thread: reads one char at a time from the transceiver and
     * tries to parse a mavlink message.
     *
     * If the message is successfully parsed, the onReceive function is
     * executed.
     */
    virtual void runReceiver(){};

    /**
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     */
    virtual void runSender(){};

    void updateQueueStats(bool appended)
    {
        miosix::Lock<miosix::FastMutex> l(mtxStatus);
        if (!appended)
        {
            LOG_ERR(logger,
                    "Buffer full, the oldest message has been discarded");
            status.nDroppedPackets++;
        }
        status.nSendQueue++;
        if (status.nSendQueue > status.maxSendQueue)
            status.maxSendQueue = status.nSendQueue;
    };

    void updateSenderStats(size_t msgCount, bool sent)
    {
        {
            miosix::Lock<miosix::FastMutex> l(mtxStatus);
            status.nSendQueue -= msgCount;
            if (!sent)
            {
                status.nSendErrors++;
                LOG_ERR(this->logger, "Could not send message");
            }
        }
        StackLogger::getInstance().updateStack(THID_MAV_SENDER);
    };

    // Transceiver used to send and receive packets.
    Transceiver* device;

    // Buffers
    static constexpr size_t MAV_IN_BUFFER_SIZE = 256;

    SyncPacketQueue<PktLength, OutQueueSize> outQueue;
    uint8_t rcvBuffer[MAV_IN_BUFFER_SIZE];

    // Status
    MavlinkStatus status;
    miosix::FastMutex mtxStatus;

    // Threads
    bool stopFlag   = false;
    bool sndStarted = false;
    bool rcvStarted = false;

    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;

    PrintLogger logger = Logging::getLogger("mavlinkdriver");
};

}  // namespace Boardcore
