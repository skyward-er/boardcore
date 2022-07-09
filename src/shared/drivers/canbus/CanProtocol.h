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

#define NPACKET 3  // equals the number of boards in the can system

/**
 * @brief Struct that contains how the canId is composed
 */
struct CanIDMask
{
    uint32_t priority    = 0x1E000000;
    uint32_t type        = 0x1F80000;
    uint32_t source      = 0x78000;
    uint32_t destination = 0x7800;
    uint32_t idType      = 0x780;
    uint32_t firstPacket = 0x40;
    uint32_t leftToSend  = 0x3F;
} idMask;

namespace Boardcore
{
namespace Canbus
{

/**
 * @brief Generic struct that contains a logical can packet
 * i.e. 1 accelerometer packet 3*4byte (acc: x,y,z)+timestamp, will be 4
 * canPacket but a single canData.
 */
struct CanData
{
    uint32_t canId =
        0;  // the id of the can packet without the last 7 bits (sequence bit)
    uint8_t len;
    uint8_t nRec = 0;
    uint64_t payload[32];
} data[NPACKET];

/**
 * @brief Canbus protocol, given an initialized can this class takes care of
 * sending the multiple packet of CanData with the corresponding id and
 * receiving single CanPacket that are then reframed as one Candata.
 */
class CanProtocol : public ActiveObject
{
private:
    miosix::FastMutex
        mutex;          // todo add mutex and create get data in can protocol
    CanbusDriver* can;  // the physical can
    IRQCircularBuffer<CanData, NPACKET>
        buffer;  // the buffer used to send data from CanProtocol to CanHandler

public:
    /**
     * @brief Construct a new CanProtocol object
     * @param can CanbusDriver pointer.
     */
    CanProtocol(CanbusDriver* can) { this->can = can; }

    CanData
    getPacket()  // return the packet, if buffer is empty return an empty packet
    {
        CanData temp;
        mutex.lock();
        if (!buffer.isEmpty())
        {
            temp = buffer.pop();
        }
        mutex.unlock();

        return temp;
    }

    bool isEmpty()
    {
        mutex.lock();
        return buffer.isEmpty();
        mutex.unlock();
    }

    void waitEmpty() { buffer.waitUntilNotEmpty(); }

    /**
     * @brief Takes a canData, it splits it into single canpacket with the
     * correct sequential id
     * @param toSend = containing the id e the data of the packet to send
     * @warning requires toSend to be not empty
     */
    void sendCan(CanData toSend)  //@requires toSen to not be empty
    {
        CanPacket packet;
        uint32_t tempLen = toSend.len - 1;
        uint32_t tempId  = toSend.canId;
        packet.ext       = true;
        packet.id =
            (tempId << 7) | idMask.firstPacket | (tempLen & idMask.leftToSend);
        packet.length = (toSend.payload[0] + 8) /
                        8;  // simple formula for upper approximation
        for (int k = 0; k < packet.length; k++)
        {
            packet.data[k] = toSend.payload[0] >> (8 * k);
        }
        tempLen--;

        can->send(packet);
        TRACE("tosend len %d\n", toSend.len);

        for (int i = 1; i < toSend.len; i++)
        {
            tempId    = toSend.canId;
            packet.id = (tempId << 7) | !(idMask.firstPacket) |
                        (tempLen & idMask.leftToSend);
            packet.length = (toSend.payload[i] + 8) / 8;
            for (int k = 0; k < packet.length; k++)
            {
                packet.data[k] = toSend.payload[i] << (8 * k);
            }
            TRACE("packetlen %d\n, dato %d\n", packet.length, packet.data[0]);
            can->send(packet);
            tempLen--;
        }
    }

protected:
    /**
     * @brief Keeps listening on hte canbus for packets, once received it checks
     * if they are expected (that id is already present in data), if they are
     * they are added to the list. once we receive the correct amount of packet
     * we send it to can handler.
     */
    void run() override  // for now if a packet is missed/received in the wrong
                         // order the whole packet will be lost once we receive
                         // a new first packet without warning canhandler
    {
        uint32_t sourceId;
        CanPacket packet;
        // Infinite loop
        while (true)
        {
            can->getRXBuffer().waitUntilNotEmpty();
            if (!can->getRXBuffer().isEmpty())
            {

                packet = can->getRXBuffer().pop().packet;

                sourceId = packet.id & idMask.source;
                if (data[sourceId].canId == 0 ||
                    (data[sourceId].canId & idMask.source) == sourceId)
                {
                    if (packet.id & idMask.firstPacket)  // it is a first
                                                         // packet of a data;
                    {
                        data[sourceId].len =
                            (packet.id & idMask.leftToSend) + 1;
                        data[sourceId].canId =
                            packet.id >> 7;  // discard the sequence number
                    }
                    TRACE("pakcet %d, nrec %d, left %lu\n", packet.data[0],
                          data[sourceId].nRec, (packet.id & idMask.leftToSend));
                    if ((data[sourceId].len - (data[sourceId].nRec + 1)) ==
                        (packet.id & idMask.leftToSend))
                    {

                        uint64_t tempPayload = 0;
                        for (int f = 0; f < packet.length; f++)
                        {
                            uint64_t tempData = packet.data[f];
                            tempPayload = tempPayload | (tempData << (f * 8));
                        }

                        data[sourceId]
                            .payload[data[sourceId].len -
                                     (packet.id & idMask.leftToSend) - 1] =
                            tempPayload;
                        data[sourceId].nRec++;
                    }

                    if (data[sourceId].nRec == data[sourceId].len)
                    {
                        mutex.lock();
                        buffer.put(data[sourceId]);
                        // empties the struct
                        data[sourceId].canId = 0;
                        data[sourceId].nRec  = 0;
                        data[sourceId].len   = 0;
                        mutex.unlock();
                    }
                }
            }
        }
    }
};
}  // namespace Canbus
}  // namespace Boardcore
