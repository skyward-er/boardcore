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
    : public MavlinkDriver<
          MavlinkDriverPignaMaster<PktLength, OutQueueSize, MavMsgLength>,
          PktLength, OutQueueSize, MavMsgLength>
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
        : MavlinkDriver<
              MavlinkDriverPignaMaster<PktLength, OutQueueSize, MavMsgLength>,
              PktLength, OutQueueSize, MavMsgLength>(device),
          device(device), onReceive(onReceive), pingMsgId(pingMsgId),
          sleepAfterSend(sleepAfterSend){};

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
        bool appended = outQueue.put(msg, size);
        // Update stats
        updateQueueStats(appended);
        // Return ok even if a packet was discarded

        if (force_flush)
        {
            std::lock_guard<std::mutex> l(mtxFlush);
            outQueue.copyClear(&sendQueue);
            condVarFlush.notify_one();
        }

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
                sndLauncher, skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
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
                rcvLauncher, skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
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
    void runReceiver() override
    {
        mavlink_message_t msg;
        ssize_t rcvSize;
        uint8_t parseResult = 0;

        while (!stopFlag)
        {
            // Check for a new message on the device
            rcvSize = device->receive(rcvBuffer, MAV_IN_BUFFER_SIZE);

            // If there's a new message ...
            if (rcvSize > 0)
            {
                parseResult = 0;
                miosix::Lock<miosix::FastMutex> l(mtxStatus);

                for (ssize_t i = 0; i < rcvSize; i++)
                {
                    // ... parse received bytes
                    parseResult = mavlink_parse_char(
                        MAVLINK_COMM_0,
                        rcvBuffer[i],         // byte to parse
                        &msg,                 // where to parse it
                        &(status.mavStats));  // stats to update

                    // When a valid message is found ...
                    if (parseResult == 1)
                    {
                        // Unlock mutex before calling the callback, no one
                        // knows what could happen.
                        miosix::Unlock<miosix::FastMutex> unlock(l);

                        LOG_DEBUG(
                            logger,
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
        LOG_DEBUG(logger, "Sender is running");
        Packet<PktLength> pkt;

        while (1)
        {

            std::unique_lock<std::mutex> l(mtxFlush);
            while (sendQueue.isEmpty())
                condVarFlush.wait(l);

            if (stopFlag)
                return;

            while (!sendQueue.isEmpty())
            {
                pkt = sendQueue.pop();  //  Remove the packet from queue
                uint64_t age =
                    TimestampTimer::getTimestamp() - pkt.getTimestamp();
                LOG_DEBUG(logger, "Sending packet. Size: {} (age: {})",
                          pkt.size(), age);
                bool sent = device->send(pkt.content.data(), pkt.size());
                miosix::Thread::sleep(sleepAfterSend);
                updateSenderStats(pkt.getMsgCount(), sent);
            }
        }
    };

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void rcvLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriverPignaMaster*>(arg)->runReceiver();
    }

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriverPignaMaster*>(arg)->runSender();
    }

    void updateSenderStats(size_t msgCount, bool sent)
    {
        {
            miosix::Lock<miosix::FastMutex> l(mtxStatus);
            status.nSendQueue -= msgCount;
            if (!sent)
            {
                status.nSendErrors++;
                LOG_ERR(logger, "Could not send message");
            }
        }
        StackLogger::getInstance().updateStack(THID_MAV_SENDER);
    };

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

    Transceiver* device;   ///< Transceiver used to send and receive packets.
    MavHandler onReceive;  ///< Function executed when a message is ready.

    // Message ID of the periodic message
    uint8_t pingMsgId;

    // Buffers
    static constexpr size_t MAV_IN_BUFFER_SIZE = 256;

    SyncPacketQueue<PktLength, OutQueueSize> outQueue;
    CircularBuffer<Packet<PktLength>, OutQueueSize> sendQueue;
    uint8_t rcvBuffer[MAV_IN_BUFFER_SIZE];

    // Status
    MavlinkStatus status;
    miosix::FastMutex mtxStatus;

    // Flush syncronization
    std::mutex mtxFlush;
    std::condition_variable condVarFlush;

    uint16_t sleepAfterSend;

    // Threads
    bool stopFlag   = false;
    bool sndStarted = false;
    bool rcvStarted = false;

    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;

    PrintLogger logger = Logging::getLogger("mavlinkdriver");
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
    : public MavlinkDriver<
          MavlinkDriverPignaSlave<PktLength, OutQueueSize, MavMsgLength>,
          PktLength, OutQueueSize, MavMsgLength>
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
                            int partialQueueSize    = OutQueueSize)
        : MavlinkDriver<
              MavlinkDriverPignaSlave<PktLength, OutQueueSize, MavMsgLength>,
              PktLength, OutQueueSize, MavMsgLength>(device),
          device(device), onReceive(onReceive), pingMsgId(pingMsgId),
          partialQueueSize(partialQueueSize), sleepAfterSend(sleepAfterSend){};

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

        return enqueueRaw(msgTempBuf, msgLen);
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
                sndLauncher, skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
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
                rcvLauncher, skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
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
    void runReceiver() override
    {
        mavlink_message_t msg;
        ssize_t rcvSize;
        uint8_t parseResult = 0;

        while (!stopFlag)
        {
            // Check for a new message on the device
            rcvSize = device->receive(rcvBuffer, MAV_IN_BUFFER_SIZE);

            // If there's a new message ...
            if (rcvSize > 0)
            {
                parseResult = 0;
                miosix::Lock<miosix::FastMutex> l(mtxStatus);

                for (ssize_t i = 0; i < rcvSize; i++)
                {
                    // ... parse received bytes
                    parseResult = mavlink_parse_char(
                        MAVLINK_COMM_0,
                        rcvBuffer[i],         // byte to parse
                        &msg,                 // where to parse it
                        &(status.mavStats));  // stats to update

                    // When a valid message is found ...
                    if (parseResult == 1)
                    {
                        // Unlock mutex before calling the callback, no one
                        // knows what could happen.
                        miosix::Unlock<miosix::FastMutex> unlock(l);

                        LOG_DEBUG(
                            logger,
                            "Received message with ID {}, sequence: {} from "
                            "component {} of system {}",
                            msg.msgid, msg.seq, msg.compid, msg.sysid);

                        // ... handle the command
                        if (onReceive != nullptr)
                            onReceive(this, msg);

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
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     */
    void runSender() override{};

    void flushMessages()
    {
        int i = 0;
        Packet<PktLength> pkt;
        while (!outQueue.isEmpty() && i < partialQueueSize)
        {
            i++;
            pkt          = outQueue.pop();  //  Remove the packet from queue
            uint64_t age = TimestampTimer::getTimestamp() - pkt.getTimestamp();
            LOG_DEBUG(logger, "Sending packet. Size: {} (age: {})", pkt.size(),
                      age);
            bool sent = device->send(pkt.content.data(), pkt.size());
            miosix::Thread::sleep(sleepAfterSend);
            updateSenderStats(pkt.getMsgCount(), sent);
        }
    };

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void rcvLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriverPignaSlave*>(arg)->runReceiver();
    }

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriverPignaSlave*>(arg)->runSender();
    }

    void updateSenderStats(size_t msgCount, bool sent)
    {
        {
            miosix::Lock<miosix::FastMutex> l(mtxStatus);
            status.nSendQueue -= msgCount;
            if (!sent)
            {
                status.nSendErrors++;
                LOG_ERR(logger, "Could not send message");
            }
        }
        StackLogger::getInstance().updateStack(THID_MAV_SENDER);
    };

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

    Transceiver* device;   ///< Transceiver used to send and receive packets.
    MavHandler onReceive;  ///< Function executed when a message is ready.

    // Message ID of the periodic message
    uint8_t pingMsgId;

    // Max number of packets to send at once
    int partialQueueSize;

    // Buffers
    static constexpr size_t MAV_IN_BUFFER_SIZE = 256;

    SyncPacketQueue<PktLength, OutQueueSize> outQueue;
    uint8_t rcvBuffer[MAV_IN_BUFFER_SIZE];

    // Status
    MavlinkStatus status;
    miosix::FastMutex mtxStatus;

    uint16_t sleepAfterSend;

    // Threads
    bool stopFlag   = false;
    bool sndStarted = false;
    bool rcvStarted = false;

    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;

    PrintLogger logger = Logging::getLogger("mavlinkdriver");
};

}  // namespace Boardcore
