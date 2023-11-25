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
    : public MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>
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
        : MavlinkDriver<PktLength, OutQueueSize, MavMsgLength>(device),
          onReceive(onReceive), sleepAfterSend(sleepAfterSend),
          outBufferMaxAge(outBufferMaxAge){};

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
     *
     * After every send, the thread sleeps to guarantee some silence on the
     * channel.
     */
    void runSender()
    {
        LOG_DEBUG(this->logger, "Sender is running");
        Packet<PktLength> pkt;

        while (!this->stopFlag)
        {
            this->outQueue.waitUntilNotEmpty();

            if (!this->outQueue.isEmpty())
            {
                // Get the first packet in the queue, without removing it
                pkt = this->outQueue.get();

                // If the packet is ready or too old, send it
                uint64_t age =
                    TimestampTimer::getTimestamp() - pkt.getTimestamp();
                if (pkt.isReady() || age >= outBufferMaxAge * 1e3)
                {
                    this->outQueue.pop();  //  Remove the packet from queue

                    LOG_DEBUG(this->logger,
                              "Sending packet. Size: {} (age: {})", pkt.size(),
                              age);

                    bool sent =
                        this->device->send(pkt.content.data(), pkt.size());
                    this->updateSenderStats(pkt.getMsgCount(), sent);

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

    MavHandler onReceive;  ///< Function executed when a message is ready.

    // Tweakable params
    uint16_t sleepAfterSend;

    size_t outBufferMaxAge;
};

}  // namespace Boardcore
