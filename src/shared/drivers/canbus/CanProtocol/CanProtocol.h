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

namespace Boardcore
{

namespace Canbus
{

/**
 * The CanProtocol allows to transmit arbitrarily sized messages over the CanBus
 * overcoming the 8 byte limitation of each single packet.
 *
 * Our CanProtocol uses the extended can packet, the 29 bits id is divided such
 * as:
 * - Priority           4 bit - priority      \
 * - Primary type       6 bit - primaryType   |
 * - Source             4 bit - source        | 22 bits - Message informations
 * - Destination        4 bit - destination   |
 * - Secondary type     4 bit - secondaryType /
 * - First packet flag  1 bit - firstPacket   \ 7 bits - Sequential informations
 * - Remaining packets  6 bit - leftToSend    /
 * shiftNameOfField the number of shift needed to reach that field
 *
 * The id is split into 2 parts:
 * - Message information: Common to every packet of a given message
 * - Sequential information: Used to distinguish between packets
 *
 * The sender splits into multiple packets a message that is then recomposed on
 * the receiver end. The message informations are encoded into the packets id,
 * therefore they have an effect on packets priorities.
 */

/**
 * @brief Masks of the elements composing can packets ids.
 */
enum class CanPacketIdMask : uint32_t
{
    PRIORITY       = 0x1E000000,
    PRIMARY_TYPE   = 0x01F80000,
    SOURCE         = 0x00078000,
    DESTINATION    = 0x00003800,
    SECONDARY_TYPE = 0x00000780,

    MESSAGE_INFORMATION = 0x1FFFFF80,

    FIRST_PACKET_FLAG = 0x00000040,
    LEFT_TO_SEND      = 0x0000003F,

    SEQUENTIAL_INFORMATION = 0x0000007F
};

enum ShiftInformation : uint8_t
{
    // Shift values for message informations
    PRIORITY       = 25,
    PRIMARY_TYPE   = 19,
    SOURCE         = 15,
    DESCRIPTION    = 11,
    SECONDARY_TYPE = 7,

    // Shift values for sequential informations
    FIRST_PACKET_FLAG = 6,
    LEFT_TO_SEND      = 0,

    // Position of the message infos relative to the entire can packet id
    SEQUENTIAL_INFORMATION = 7
};

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
    CanProtocol(CanbusDriver* can, MsgHandler onReceive);

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
     * @brief Non-blocking send function, puts the messages in a queue.
     * Message is discarded if the queue is full.
     *
     * @param msg Message to send (CanMessage struct).
     * @return True if the message could be enqueued.
     */
    bool enqueueMsg(const CanMessage& msg);

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

    SyncCircularBuffer<CanMessage, 10> outQueue;

    PrintLogger logger = Logging::getLogger("canprotocol");
};

}  // namespace Canbus

}  // namespace Boardcore
