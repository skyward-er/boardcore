/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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
#include <drivers/canbus/CanDriver/CanDriver.h>
#include <utils/Debug.h>
#include <utils/collections/SyncCircularBuffer.h>

#include "CanProtocolData.h"
#include "CanProtocolTypes.h"

namespace Boardcore
{

namespace Canbus
{

/**
 * @brief Canbus protocol implementation.
 *
 * Given a can interface this class takes care of sending messages segmented
 * into multiple packets, and receiving single packets that are then reframed
 * into messages.
 *
 * This driver has been implemented following the MavlinkDriver.
 */
class CanProtocol
{
    using MsgHandler = std::function<void(CanMessage data)>;

public:
    /**
     * @brief Construct a new CanProtocol object.
     *
     * @param can Pointer to a CanbusDriver object.
     */
    CanProtocol(CanbusDriver* can, MsgHandler onReceive,
                miosix::Priority threadPriority);

    /**
     * @brief Start the receiving and sending threads.
     *
     * @return False if at least one could not start.
     */
    bool start();

    /**
     * @brief Tells whether the driver was started.
     */
    bool isStarted();

    /**
     * @brief Stops sender and receiver threads.
     */
    void stop();

    /**
     * @brief Adds a filter to the can peripheral to receive only messages from
     * the given source and targeted to the given destination.
     *
     * @param src Message source.
     * @param dst Message destination.
     * @return True if the filter was added successfully.
     */
    bool addFilter(uint8_t src, uint64_t dst);

    /**
     * @brief Non-blocking send function, puts the messages in a queue.
     * Message is discarded if the queue is full.
     *
     * @param msg Message to send (CanMessage struct).
     * @return True if the message could be enqueued.
     */
    bool enqueueMsg(const CanMessage& msg);

    /**
     * @brief Non-blocking send function for an event (a message without
     * payload).
     */
    bool enqueueEvent(uint8_t priority, uint8_t primaryType, uint8_t source,
                      uint8_t destination, uint8_t secondaryType);

    /**
     * @brief Non-blocking send function for a simple packet with a payload of 1
     * can packet, useful to send generic events/commands with a short payload
     * without using an encoding/decoding function.
     */
    bool enqueueSimplePacket(uint8_t priority, uint8_t primaryType,
                             uint8_t source, uint8_t destination,
                             uint8_t secondaryType, uint64_t payload);

    /**
     * @brief Non-blocking send function for a generic data type.
     *
     * @warning There must be a function called with this prototype:
     *   CanMessage toCanMessage(const T& t);
     *
     * @param t The class to be logged.
     */
    template <typename T>
    bool enqueueData(uint8_t priority, uint8_t primaryType, uint8_t source,
                     uint8_t destination, uint8_t secondaryType, const T& t);

private:
    /**
     * @brief Blocking send function, puts the CanMessage object on the bus.
     *
     * Takes a CanMessage object, splits it into multiple CanPackets with the
     * correct sequential id.
     *
     * @param msg Contains the id and the data of the packet to send.
     */
    void sendMessage(const CanMessage& msg);

    /**
     * @brief Receiver thread: waits for one packet at a time from the can
     * driver and tries to parse a message.
     *
     * If the message is successfully parsed, the onReceive function is
     * executed.
     *
     * For now if a packet is received in the wrong order or if a packet with a
     * different id is received, the current (incomplete) message will be lost.
     * Once we receive a new first packet, currently saved data are reset.
     */
    void runReceiver();

    /**
     * @brief Sender Thread: Periodically flushes the message queue and sends
     * all the enqueued messages.
     */
    void runSender();

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void rcvLauncher(void* arg);

    /**
     * @brief Calls the run member function.
     *
     * @param arg The object pointer cast to void*.
     */
    static void sndLauncher(void* arg);

    /**
     * @brief Count the number of bytes needed to encode a uint64_t number.
     */
    uint8_t byteForUint64(uint64_t number);

    CanbusDriver* can;     ///< Device used to send and receive packets.
    MsgHandler onReceive;  ///< Function executed when a message is ready.

    // Threads
    bool stopFlag   = false;
    bool sndStarted = false;
    bool rcvStarted = false;

    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;
    miosix::Priority threadPriority;

    SyncCircularBuffer<CanMessage, 10> outQueue;

    PrintLogger logger = Logging::getLogger("canprotocol");
};

template <typename T>
bool CanProtocol::enqueueData(uint8_t priority, uint8_t primaryType,
                              uint8_t source, uint8_t destination,
                              uint8_t secondaryType, const T& t)
{
    if (priority > 0xF || primaryType > 0x3F || source > 0xF ||
        destination > 0xF || secondaryType > 0xF)
        return false;

    CanMessage msg = toCanMessage(t);

    // clang-format off
    msg.id =  priority      << static_cast<uint32_t>(CanProtocolShiftInformation::PRIORITY);
    msg.id |= primaryType   << static_cast<uint32_t>(CanProtocolShiftInformation::PRIMARY_TYPE);
    msg.id |= source        << static_cast<uint32_t>(CanProtocolShiftInformation::SOURCE);
    msg.id |= destination   << static_cast<uint32_t>(CanProtocolShiftInformation::DESTINATION);
    msg.id |= secondaryType << static_cast<uint32_t>(CanProtocolShiftInformation::SECONDARY_TYPE);
    // clang-format off

    return enqueueMsg(msg);
}

}  // namespace Canbus

}  // namespace Boardcore
