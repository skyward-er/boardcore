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
#include <drivers/Transceiver.h>
#include <mavlink_lib/mavlink_types.h>
#include <utils/collections/SyncPacketQueue.h>

#include "MavlinkStatus.h"

namespace Boardcore
{

/**
 * @brief The **MavChannel** object offers an interface to send and receive from
 * a Transceiver object using an implementation of the Mavlink protocol.
 * See `tests/mavlink/test-mavlink.cpp` for an axample of usage.
 *
 * @tparam PktLength         Maximum length in bytes of each packet.
 * @tparam OutQueueSize  Max number of packets in the out queue.
 */
template <unsigned int PktLength, unsigned int OutQueueSize>
class MavlinkDriver
{
    /* Alias of the function to be executed on message rcv */
    using MavHandler = std::function<void(MavlinkDriver* channel,
                                          const mavlink_message_t& msg)>;

public:
    /**
     * @param device     transceiver used to send and receive messages
     * @param onRcv      function to be executed on message rcv
     * @param sleepAfterSend  guaranteed sleep time after each send (ms)
     * @param outBufferMaxAge max residence time for message in the queue: after
     *                        this time the message will be automatically sent.
     */
    MavlinkDriver(Transceiver* device, MavHandler onRcv,
                  uint16_t sleepAfterSend = 0, size_t outBufferMaxAge = 1000);

    ~MavlinkDriver() {}

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
     * @param msg    message to send (mavlink struct)
     * @return true if the message could be enqueued (queue not full)
     */
    bool enqueueMsg(const mavlink_message_t& msg);

    /**
     * @brief Receiver thread: reads one char at a time from the transceiver and
     * tries to parse a mavlink message.
     * If the message is successfully parsed, the onRcv function is executed.
     */
    void runReceiver();
    /**
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     * After every send, the thread sleeps to guarantee some silence on the
     * channel.
     */
    void runSender();

    /**
     * @brief Synchronized status getter
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
     * @brief Calls the run member function
     * @param arg the object pointer cast to void*
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavlinkDriver*>(arg)->runSender();
    }

    void updateQueueStats(int dropped);

    void updateSenderStats(size_t msgCount, bool sent);

    Transceiver* device;  // transceiver used to send and receive
    MavHandler onRcv;     // function executed on message rcv

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

/**********************************************************************
 * NOTE: Dont't move the implementation in a .cpp, it won't compile.  *
 **********************************************************************/
template <unsigned int PktLength, unsigned int OutQueueSize>
MavlinkDriver<PktLength, OutQueueSize>::MavlinkDriver(Transceiver* device,
                                                      MavHandler onRcv,
                                                      uint16_t sleepAfterSend,
                                                      size_t outBufferMaxAge)
    : device(device), onRcv(onRcv), sleepAfterSend(sleepAfterSend),
      outBufferMaxAge(outBufferMaxAge)
{
    memset(&status, 0, sizeof(MavlinkStatus));
}

template <unsigned int PktLength, unsigned int OutQueueSize>
bool MavlinkDriver<PktLength, OutQueueSize>::start()
{
    stopFlag = false;

    // Start sender (joinable thread)
    if (!sndStarted)
    {
        sndThread = miosix::Thread::create(
            sndLauncher, skywardStack(4 * 1024), miosix::MAIN_PRIORITY,
            reinterpret_cast<void*>(this), miosix::Thread::JOINABLE);

        if (sndThread != nullptr)
        {
            sndStarted = true;
        }
        else
        {
            LOG_ERR(logger, "Could not start sender!");
        }
    }

    // Start receiver
    if (!rcvStarted)
    {
        rcvThread = miosix::Thread::create(rcvLauncher, skywardStack(4 * 1024),
                                           miosix::MAIN_PRIORITY,
                                           reinterpret_cast<void*>(this));

        if (rcvThread != nullptr)
        {
            rcvStarted = true;
        }
        else
        {
            LOG_ERR(logger, "Could not start receiver!");
        }
    }

    if (sndStarted && rcvStarted)
    {
        LOG_DEBUG(logger, "Start ok (sender and receiver)\n");
    }

    return (sndStarted && rcvStarted);
}

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::stop()
{
    stopFlag = true;

    // Wait for sender to stop
    sndThread->join();
}

template <unsigned int PktLength, unsigned int OutQueueSize>
bool MavlinkDriver<PktLength, OutQueueSize>::enqueueMsg(
    const mavlink_message_t& msg)
{
    // Convert mavlink message to char array
    uint8_t msgtempBuf[PktLength];
    int msgLen = mavlink_msg_to_send_buffer(msgtempBuf, &msg);

    // Append message to the queue
    int dropped = outQueue.put(msgtempBuf, msgLen);

    // Update stats
    updateQueueStats(dropped);

    // return ok even if a packet was discarded
    return dropped != -1;
}

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::updateQueueStats(int dropped)
{
    miosix::Lock<miosix::FastMutex> l(mtxStatus);

    if (dropped != 0)
    {
        LOG_ERR(logger, "Buffer full, the oldest message has been discarded");
        status.nDroppedPackets++;
    }

    status.nSendQueue++;

    if (status.nSendQueue > status.maxSendQueue)
    {
        status.maxSendQueue = status.nSendQueue;
    }
}

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::runReceiver()
{
    mavlink_message_t msg;
    ssize_t rcvSize;
    uint8_t parseResult = 0;

    while (!stopFlag)
    {
        // Check for a new message on the device
        rcvSize = device->receive(rcvBuffer, MAV_IN_BUFFER_SIZE);

        // If there's a new messages...
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
                    onRcv(this, msg);

                    StackLogger::getInstance().updateStack(THID_MAV_RECEIVER);
                }
            }
        }
    }
}

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::runSender()
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

            uint64_t age = (uint64_t)miosix::getTick() - pkt.timestamp();
            // If the packet is ready or too old, send it
            if (pkt.isReady() || age >= outBufferMaxAge)
            {
                outQueue.pop();  // remove from queue

                // LOG_DEBUG(logger, "Sending packet. Size: {} (age: {})",
                //           pkt.size(), age);

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

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::updateSenderStats(size_t msgCount,
                                                               bool sent)
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

template <unsigned int PktLength, unsigned int OutQueueSize>
MavlinkStatus MavlinkDriver<PktLength, OutQueueSize>::getStatus()
{
    miosix::Lock<miosix::FastMutex> l(mtxStatus);
    status.timestamp = miosix::getTick();
    return status;
}

template <unsigned int PktLength, unsigned int OutQueueSize>
void MavlinkDriver<PktLength, OutQueueSize>::setSleepAfterSend(
    uint16_t newSleepTime)
{
    sleepAfterSend = newSleepTime;
}

}  // namespace Boardcore
