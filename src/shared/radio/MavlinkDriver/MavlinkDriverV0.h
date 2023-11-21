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
#include "MavlinkDriver.h"

namespace Boardcore
{

/**
 * @brief The Default implementation of MavlinkDriver
 *
 * See `src/tests/drivers/test-mavlink-v0.cpp` for an example.
 *
 * @tparam PktLength Maximum length in bytes of each transceiver packet.
 * @tparam OutQueueSize Max number of transceiver packets in the output queue.
 * @tparam MavMsgLength Max length of a mavlink message. By default is 255 the
 * @tparam sleepAfterSend Guaranteed sleep time after each send [ms].
 * maximum possible but can be replaces with MAVLINK_MAX_DIALECT_PAYLOAD_SIZE
 * for a specific protocol.
 */
template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength = MAVLINK_MAX_PAYLOAD_LEN>
class MavlinkDriverV0
    : public MavlinkDriver<
          MavlinkDriverV0<PktLength, OutQueueSize, MavMsgLength>, PktLength,
          OutQueueSize, MavMsgLength>
{
    /// Alias of the function to be executed on message reception.
    using MavHandler = std::function<void(MavlinkDriverV0* channel,
                                          const mavlink_message_t& msg)>;

public:
    /**
     * @brief Initializes all data structures.
     *
     * @param device Transceiver used to send and receive messages.
     * @param onReceive Function to be executed on message rcv.
     * @param sleepAfterSend Guaranteed sleep time after each send [ms].
     * @param outBufferMaxAge Max residence time for messages in the queue:
     * after this time the message will be automatically sent [ms].
     */
    MavlinkDriverV0(Transceiver* device, MavHandler onReceive = nullptr,
                    uint16_t sleepAfterSend = 0, size_t outBufferMaxAge = 1000)
        : MavlinkDriver<MavlinkDriverV0<PktLength, OutQueueSize, MavMsgLength>,
                        PktLength, OutQueueSize, MavMsgLength>(device),
          device(device), onReceive(onReceive), sleepAfterSend(sleepAfterSend),
          outBufferMaxAge(outBufferMaxAge){};

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
    void runReceiver()
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
     *
     * After every send, the thread sleeps to guarantee some silence on the
     * channel.
     */
    void runSender()
    {
        LOG_DEBUG(logger, "Sender is running");
        Packet<PktLength> pkt;

        while (!stopFlag)
        {
            outQueue.waitUntilNotEmpty();

            if (!outQueue.isEmpty())
            {
                // Get the first packet in the queue, without removing it
                pkt = outQueue.get();

                // If the packet is ready or too old, send it
                uint64_t age =
                    TimestampTimer::getTimestamp() - pkt.getTimestamp();
                if (pkt.isReady() || age >= outBufferMaxAge * 1e3)
                {
                    outQueue.pop();  //  Remove the packet from queue

                    LOG_DEBUG(logger, "Sending packet. Size: {} (age: {})",
                              pkt.size(), age);

                    bool sent = device->send(pkt.content.data(), pkt.size());
                    updateSenderStats(pkt.getMsgCount(), sent);

                    miosix::Thread::sleep(sleepAfterSend);
                }
                else
                {
                    // Wait before sending something else
                    miosix::Thread::sleep(50);
                }
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
        reinterpret_cast<MavlinkDriverV0*>(arg)->runReceiver();
    }

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriverV0*>(arg)->runSender();
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

    // Tweakable params
    uint16_t sleepAfterSend;
    size_t outBufferMaxAge;

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
