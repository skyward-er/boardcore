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

#include <diagnostic/PrintLogger.h>

#include <vector>

/**
 * This object includes only the protocol header (`protocol.h`). To use this
 * driver, you should include YOUR OWN implementation of the messages definition
 * (`mavlink.h`) before including this header. To create your implementation you
 * can use Skyward's *Mavlink Editor*.
 */
#ifndef MAVLINK_H
#error \
    "[MavlinkDriver] Mavlink header not found! Please include your mavlink.h \
implementation before including MavlinkDriver.h"
#endif

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
 * maximun possible but can be replaces with MAVLINK_MAX_DIALECT_PAYLOAD_SIZE
 * for a specific protocol.
 */
template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength = MAVLINK_MAX_PAYLOAD_LEN>
class MavlinkDriver
{
    ///< Alias of the function to be executed on message reception.
    using MavHandler = std::function<void(MavlinkDriver* channel,
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
    MavlinkDriver(Transceiver* device, MavHandler onReceive = nullptr,
                  uint16_t sleepAfterSend = 0, size_t outBufferMaxAge = 1000);

    /**
     * @brief  Start the receiving and sending threads.
     * @return false if at least one could not start.
     */
    bool start();

    /**
     * @brief Stops sender and receiver threads.
     */
    void stop();

    /**
     * @brief Non-blocking send function, puts the message in a queue.
     * Message is discarded if the queue is full.
     *
     * @param msg Message to send (mavlink struct).
     * @return True if the message could be enqueued.
     */
    bool enqueueMsg(const mavlink_message_t& msg);

    /**
     * @brief Enqueue a raw packet message into the sync packet queue.
     *
     * @param msg Messa to send.
     * @param size Length in bytes.
     * @return True if the message was enqueued.
     */
    bool enqueueRaw(uint8_t* msg, size_t size);

    /**
     * @brief Receiver thread: reads one char at a time from the transceiver and
     * tries to parse a mavlink message.
     *
     * If the message is successfully parsed, the onReceive function is
     * executed.
     */
    void runReceiver();

    /**
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     *
     * After every send, the thread sleeps to guarantee some silence on the
     * channel.
     */
    void runSender();

    /**
     * @brief Synchronized status getter.
     */
    MavlinkStatus getStatus();

    /**
     * @brief Setter for the sleep after send value.
     */
    void setSleepAfterSend(uint16_t newSleepTime);

private:
    /**
     * @brief Calls the run member function
     * @param arg the object pointer cast to void*
     */
    static void rcvLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriver*>(arg)->runReceiver();
    }

    /**
     * @brief Calls the run member function.
     *
     * @param arg the object pointer cast to void*
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriver*>(arg)->runSender();
    }

    void updateQueueStats(bool appended);

    void updateSenderStats(size_t msgCount, bool sent);

    Transceiver* device;   ///< transceiver used to send and receive
    MavHandler onReceive;  ///< function executed on message rcv

    // Tweakable params
    uint16_t sleepAfterSend;
    size_t outBufferMaxAge;
    uint16_t pollingTime = 100;  // ms

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

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::MavlinkDriver(
    Transceiver* device, MavHandler onReceive, uint16_t sleepAfterSend,
    size_t outBufferMaxAge)
    : device(device), onReceive(onReceive), sleepAfterSend(sleepAfterSend),
      outBufferMaxAge(outBufferMaxAge)
{
    memset(&status, 0, sizeof(MavlinkStatus));
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
bool MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::start()
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
        rcvThread = miosix::Thread::create(rcvLauncher, skywardStack(4 * 1024),
                                           miosix::MAIN_PRIORITY,
                                           reinterpret_cast<void*>(this));

        if (rcvThread != nullptr)
            rcvStarted = true;
        else
            LOG_ERR(logger, "Could not start receiver!");
    }

    if (sndStarted && rcvStarted)
        LOG_DEBUG(logger, "Sender and receiver started");

    return (sndStarted && rcvStarted);
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::stop()
{
    stopFlag = true;

    // Wait for sender to stop
    sndThread->join();
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
bool MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::enqueueMsg(
    const mavlink_message_t& msg)
{
    // Convert mavlink message to a byte array
    uint8_t msgtempBuf[MAVLINK_NUM_NON_PAYLOAD_BYTES + MavMsgLength];
    int msgLen = mavlink_msg_to_send_buffer(msgtempBuf, &msg);

    // Append message to the queue
    bool appended = outQueue.put(msgtempBuf, msgLen);

    // Update stats
    updateQueueStats(appended);

    // Return ok even if a packet was discarded
    return appended;
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
bool MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::enqueueRaw(
    uint8_t* msg, size_t size)
{
    // Append message to the queue
    bool appended = outQueue.put(msg, size);

    // Update stats
    updateQueueStats(appended);

    // Return ok even if a packet was discarded
    return appended;
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::updateQueueStats(
    bool appended)
{
    miosix::Lock<miosix::FastMutex> l(mtxStatus);

    if (!appended)
    {
        LOG_ERR(logger, "Buffer full, the oldest message has been discarded");
        status.nDroppedPackets++;
    }

    status.nSendQueue++;

    if (status.nSendQueue > status.maxSendQueue)
        status.maxSendQueue = status.nSendQueue;
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::runReceiver()
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
                parseResult =
                    mavlink_parse_char(MAVLINK_COMM_0,
                                       rcvBuffer[i],  // byte to parse
                                       &msg,          // where to parse it
                                       &(status.mavStats));  // stats to update

                // When a valid message is found ...
                if (parseResult == 1)
                {
                    // Unlock mutex before calling the callback, no one knows
                    // what could happen.
                    miosix::Unlock<miosix::FastMutex> unlock(l);

                    LOG_DEBUG(logger,
                              "Received message with ID {}, sequence: {} from "
                              "component {} of system {}",
                              msg.msgid, msg.seq, msg.compid, msg.sysid);

                    // ... handle the command
                    if (onReceive != nullptr)
                        onReceive(this, msg);

                    StackLogger::getInstance().updateStack(THID_MAV_RECEIVER);
                }
            }
        }
    }
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::runSender()
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
            uint64_t age = TimestampTimer::getTimestamp() - pkt.getTimestamp();
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
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::updateSenderStats(
    size_t msgCount, bool sent)
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
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
MavlinkStatus MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::getStatus()
{
    miosix::Lock<miosix::FastMutex> l(mtxStatus);
    status.timestamp = miosix::getTick();
    return status;
}

template <unsigned int PktLength, unsigned int OutQueueSize,
          unsigned int MavMsgLength>
void MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>::setSleepAfterSend(
    uint16_t newSleepTime)
{
    sleepAfterSend = newSleepTime;
}

}  // namespace Boardcore
