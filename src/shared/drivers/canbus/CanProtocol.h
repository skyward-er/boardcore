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

#include <ActiveObject.h>
#include <utils/Debug.h>
#include <utils/collections/IRQCircularBuffer.h>

#include "Canbus.h"

#define N_PACKET 3  ///< Number of boards on the bus.

namespace Boardcore
{

namespace Canbus
{

/**
 * The id of a can packet is composed of 29 bits and will be divided such as:
 * - Priority           4 bit - priority    \
 * - Type               6 bit - type        |
 * - Source             4 bit - source      | 22 bits
 * - Destination        4 bit - destination |
 * - Type id            4 bit - idType      /
 * - First packet flag  1 bit - firstPacket \ 7 bits
 * - Remaining packets  6 bit - leftToSend  /
 * shiftNameOfField the number of shift needed to reach that field
 */

/**
 * @brief The mask of the ID without the sequential information.
 *
 * CompleteID = (IDMask << shiftSequentialInfo) || SequentialInformation
 */

/**
 * @brief Enumeration that contains masks of the elements composing the can
 * packet id without sequential information.
 */
enum IDMask
{
    priority         = 0x3C0000,
    shiftPriority    = 18,
    type             = 0x03F000,
    shiftType        = 12,
    source           = 0x000F00,
    shiftSource      = 8,
    destination      = 0x0000F0,
    shiftDestination = 4,
    idType           = 0x00000F,
    shiftIdType      = 0
};

/**
 * @brief @brief Enumeration that contains masks of the elements composing the
 * sequential information.
 */
enum SequentialInformation
{
    firstPacket         = 0x40,
    shiftFirstPacket    = 6,
    leftToSend          = 0x3F,
    shiftLeftToSend     = 0,
    shiftSequentialInfo = 7
};

/**
 * @brief Generic struct that contains a logical can packet.
 *
 * i.e. 1 accelerometer packet 3 * 4byte (acc: x,y,z)  +timestamp, will be 4
 * canPacket but a single canData.
 */
struct CanData
{
    int32_t canId = -1;  ///< Id of the packet without the sequential info.
    uint8_t length;
    uint8_t nRec = 0;
    uint64_t payload[32];
};

/**
 * @brief Canbus protocol implementation.
 *
 * Given a can interface this class takes care of sending CanData packets
 * segmented into multiple CanPackets and receiving single CanPackets that are
 * then reframed as CanData.
 */
class CanProtocol : public ActiveObject
{
private:
    // TODO: Add mutex and create get data in can protocol
    miosix::FastMutex mutex;
    CanbusDriver* can;  // the physical can
    IRQCircularBuffer<CanData, N_PACKET>
        buffer;  // the buffer used to send data from CanProtocol to CanHandler

public:
    /**
     * @brief Construct a new CanProtocol object.
     *
     * @param can Pointer to a CanbusDriver object.
     */
    explicit CanProtocol(CanbusDriver* can);

    ~CanProtocol();

    /**
     * @brief Returns the first packet in the buffer.
     *
     * If buffer is empty return an empty packet.
     * @warning Should be called only after checking isEmpty()
     */
    CanData getPacket();

    bool isBufferEmpty();

    void waitBufferEmpty();

    /**
     * @brief Sends a CanData object on the bus.
     *
     * Takes a CanData object, splits it into multiple CanPackets with the
     * correct sequential id.
     * @warning requires @param data to be not empty.
     *
     * @param data Contains the id e the data of the packet to send.
     */
    void sendData(CanData dataToSend);

private:
    /**
     * @brief Count the number of bytes needed to encode a uint64_t number.
     */
    uint8_t byteForInt(uint64_t number);

    /**
     * @brief Keeps listening on the canbus for packets.
     *
     * Once a packet is received, it checks if it is expected (that id is
     * already present in data), if that is the case, it is added to the list.
     * Once we receive the correct amount of packet we send it to can handler.
     *
     * For now if a packet is missed/received in the wrong order the whole
     * packet will be lost once we receive a new first packet without warning
     * CanHandler.
     */
    void run() override;
};

}  // namespace Canbus

}  // namespace Boardcore
