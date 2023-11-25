/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <condition_variable>

#include "MavlinkDriver.h"

namespace Boardcore
{

/**
 * @brief A MavlinkDriver which implements the master portion of the Pigna
 * protocol
 *
 * See `src/tests/drivers/test-mavlink-pigna-master.cpp` for an example.
 *
 * @tparam PktLength Maximum length in bytes of each transceiver packet.
 * @tparam OutQueueSize Max number of transceiver packets in the output queue.
 * @tparam MavMsgLength Max length of a mavlink message. By default is 255 the
 * maximum possible but can be replaces with MAVLINK_MAX_DIALECT_PAYLOAD_SIZE
 * for a specific protocol.
 */
template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength = MAVLINK_MAX_PAYLOAD_LEN>
class MavlinkDriverPignaMaster
    : public MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>
{

    /// Alias of the function to be executed on message reception.
    using MavHandler = std::function<void(MavlinkDriverPignaMaster* channel,
                                          const mavlink_message_t& msg)>;

public:
    /**
     * @brief Initializes all data structures.
     *
     * @param device Transceiver used to send and receive messages.
     * @param onReceive Function to be executed on message rcv.
     * @param outBufferMaxAge Max residence time for messages in the queue:
     * after this time the message will be automatically sent [ms].
     */
    MavlinkDriverPignaMaster(Transceiver* device, uint8_t pingMsgId,
                             MavHandler onReceive    = nullptr,
                             uint16_t sleepAfterSend = 0)
        : MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>(device),
          onReceive(onReceive), pingMsgId(pingMsgId),
          sleepAfterSend(sleepAfterSend){};

    /**
     * @brief Non-blocking send function, puts the message in a queue.
     * Message is discarded if the queue is full.
     *
     * @param msg Message to send (mavlink struct).
     * @return True if the message could be enqueued.
     */
    bool enqueueMsg(const mavlink_message_t& msg, bool force_flush = false)
    {
        // Convert mavlink message to a byte array
        uint8_t msgTempBuf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MavMsgLength];
        int msgLen = mavlink_msg_to_send_buffer(msgTempBuf, &msg);
        bool flush = (msg.msgid == pingMsgId) || force_flush;
        return enqueueRaw(msgTempBuf, msgLen, flush);
    };

    /**
     * @brief Enqueue a raw packet message into the sync packet queue.
     *
     * @param msg Message to send.
     * @param size Length in bytes.
     * @return True if the message was enqueued.
     */
    bool enqueueRaw(uint8_t* msg, size_t size, bool force_flush = false)
    {
        // Append message to the queue
        bool appended = this->outQueue.put(msg, size);
        // Update stats
        this->updateQueueStats(appended);
        // Return ok even if a packet was discarded

        if (force_flush)
        {
            std::lock_guard<std::mutex> l(mtxFlush);
            this->outQueue.copyClear(&sendQueue);
            condVarFlush.notify_one();
        }

        return appended;
    };

protected:
    /**
     * @brief Receiver thread: reads one char at a time from the transceiver and
     * tries to parse a mavlink message.
     *
     * If the message is successfully parsed, the onReceive function is
     * executed.
     */
    void runReceiver() override
    {
        mavlink_message_t msg;
        ssize_t rcvSize;
        uint8_t parseResult = 0;

        while (!this->stopFlag)
        {
            // Check for a new message on the device
            rcvSize = this->device->receive(this->rcvBuffer,
                                            this->MAV_IN_BUFFER_SIZE);

            // If there's a new message ...
            if (rcvSize > 0)
            {
                parseResult = 0;
                std::unique_lock<std::mutex> l(this->mtxStatus);

                for (ssize_t i = 0; i < rcvSize; i++)
                {
                    // ... parse received bytes
                    parseResult = mavlink_parse_char(
                        MAVLINK_COMM_0,
                        this->rcvBuffer[i],         // byte to parse
                        &msg,                       // where to parse it
                        &(this->status.mavStats));  // stats to update

                    // When a valid message is found ...
                    if (parseResult == 1)
                    {
                        // Unlock mutex before calling the callback, no one
                        // knows what could happen.
                        l.unlock();

                        LOG_DEBUG(
                            this->logger,
                            "Received message with ID {}, sequence: {} from "
                            "component {} of system {}",
                            msg.msgid, msg.seq, msg.compid, msg.sysid);

                        // ... handle the command
                        if (onReceive != nullptr)
                            onReceive(this, msg);

                        StackLogger::getInstance().updateStack(
                            THID_MAV_RECEIVER);
                    }
                }
            }
        }
    };

    /**
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     */
    void runSender() override
    {
        LOG_DEBUG(this->logger, "Sender is running");
        Packet<PktLength> pkt;

        while (1)
        {

            std::unique_lock<std::mutex> l(mtxFlush);
            while (sendQueue.isEmpty())
                condVarFlush.wait(l);

            if (this->stopFlag)
                return;

            while (!sendQueue.isEmpty())
            {
                pkt = sendQueue.pop();  //  Remove the packet from queue
                uint64_t age =
                    TimestampTimer::getTimestamp() - pkt.getTimestamp();
                LOG_DEBUG(this->logger, "Sending packet. Size: {} (age: {})",
                          pkt.size(), age);
                bool sent = this->device->send(pkt.content.data(), pkt.size());
                miosix::Thread::sleep(sleepAfterSend);
                this->updateSenderStats(pkt.getMsgCount(), sent);
            }
        }
    };

    MavHandler onReceive;  ///< Function executed when a message is ready.

    CircularBuffer<Packet<PktLength>, OutQueueSize> sendQueue;

    // Flush syncronization
    std::mutex mtxFlush;
    std::condition_variable condVarFlush;

    // Message ID of the periodic message
    uint8_t pingMsgId;

    // Max number of packets to send at once
    int partialQueueSize;

    // Sleep time in milliseconds after a packet is sent
    uint16_t sleepAfterSend;
};

/**
 * @brief A MavlinkDriver which implements the slave portion of the Pigna
 * protocol
 *
 * See `src/tests/drivers/test-mavlink-pigna-slave.cpp` for an example.
 *
 * @tparam PktLength Maximum length in bytes of each transceiver packet.
 * @tparam OutQueueSize Max number of transceiver packets in the output queue.
 * @tparam MavMsgLength Max length of a mavlink message. By default is 255 the
 * maximum possible but can be replaces with MAVLINK_MAX_DIALECT_PAYLOAD_SIZE
 * for a specific protocol.
 */
template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength = MAVLINK_MAX_PAYLOAD_LEN>
class MavlinkDriverPignaSlave
    : public MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>
{
    /// Alias of the function to be executed on message reception.
    using MavHandler = std::function<void(MavlinkDriverPignaSlave* channel,
                                          const mavlink_message_t& msg)>;

public:
    /**
     * @brief Initializes all data structures.
     *
     * @param device Transceiver used to send and receive messages.
     * @param onReceive Function to be executed on message rcv.
     * @param outBufferMaxAge Max residence time for messages in the queue:
     * after this time the message will be automatically sent [ms].
     */
    MavlinkDriverPignaSlave(Transceiver* device, uint8_t pingMsgId,
                            MavHandler onReceive    = nullptr,
                            uint16_t sleepAfterSend = 0,
                            uint16_t timeout        = 5000,
                            int partialQueueSize    = OutQueueSize)
        : MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>(device),
          onReceive(onReceive), pingMsgId(pingMsgId),
          partialQueueSize(partialQueueSize), sleepAfterSend(sleepAfterSend),
          timeout(timeout), timeout_tick(timeout){};

protected:
    /**
     * @brief Receiver thread: reads one char at a time from the transceiver and
     * tries to parse a mavlink message.
     *
     * If the message is successfully parsed, the onReceive function is
     * executed.
     */
    void runReceiver() override
    {
        mavlink_message_t msg;
        ssize_t rcvSize;
        uint8_t parseResult = 0;

        while (!this->stopFlag)
        {
            // Check for a new message on the device
            rcvSize = this->device->receive(this->rcvBuffer,
                                            this->MAV_IN_BUFFER_SIZE);

            // If there's a new message ...
            if (rcvSize > 0)
            {
                parseResult = 0;
                std::unique_lock<std::mutex> l(this->mtxStatus);

                for (ssize_t i = 0; i < rcvSize; i++)
                {
                    // ... parse received bytes
                    parseResult = mavlink_parse_char(
                        MAVLINK_COMM_0,
                        this->rcvBuffer[i],         // byte to parse
                        &msg,                       // where to parse it
                        &(this->status.mavStats));  // stats to update

                    // When a valid message is found ...
                    if (parseResult == 1)
                    {

                        // Reset the timeout
                        timeout_tick = miosix::getTick() + timeout;

                        // Unlock mutex before calling the callback, no one
                        // knows what could happen.
                        l.unlock();

                        LOG_DEBUG(
                            this->logger,
                            "Received message with ID {}, sequence: {} from "
                            "component {} of system {}",
                            msg.msgid, msg.seq, msg.compid, msg.sysid);

                        // ... handle the command
                        if (onReceive != nullptr)
                            onReceive(this, msg);

                        // We are allowed to talk, flush all messages
                        if (msg.msgid == pingMsgId)
                        {
                            flushMessages();
                        }

                        StackLogger::getInstance().updateStack(
                            THID_MAV_RECEIVER);
                    }
                }
            }
        }
    };

    /**
     * @brief Sender Thread: If we haven't received from the master for some
     * time flush all the messages
     */
    void runSender() override
    {
        while (!this->stopFlag)
        {

            miosix::Thread::sleepUntil(timeout_tick);

            // Mavlink has stopped while waiting
            if (this->stopFlag)
                return;

            // Too much time has passed without hearing: flush now.
            if (miosix::getTick() >= timeout_tick)
            {
                flushMessages();
                timeout_tick = miosix::getTick() + timeout;
            }
        }
    }

    /**
     * @brief Flush the packets contained in our queue. The maximum number of
     * packets sent is controlled by partialQueueSize
     */
    void flushMessages()
    {
        std::lock_guard<std::mutex> l(mtxFlush);
        int i = 0;
        Packet<PktLength> pkt;
        while (!this->outQueue.isEmpty() && i < partialQueueSize)
        {
            i++;
            pkt = this->outQueue.pop();  //  Remove the packet from queue
            uint64_t age = TimestampTimer::getTimestamp() - pkt.getTimestamp();
            LOG_DEBUG(this->logger, "Sending packet. Size: {} (age: {})",
                      pkt.size(), age);
            bool sent = this->device->send(pkt.content.data(), pkt.size());
            miosix::Thread::sleep(sleepAfterSend);
            this->updateSenderStats(pkt.getMsgCount(), sent);
        }
    };

    MavHandler onReceive;  ///< Function executed when a message is ready.

    // Message ID of the periodic message
    uint8_t pingMsgId;

    // Max number of packets to send at once
    int partialQueueSize;

    // Sleep time in milliseconds after a packet is sent
    uint16_t sleepAfterSend;

    // Flush syncronization
    std::mutex mtxFlush;
    uint16_t timeout;
    uint16_t timeout_tick;
};

}  // namespace Boardcore
