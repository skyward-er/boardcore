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

// this was written under the influence of redbull and ichnusa, pls do not judge

#pragma once

#include <ActiveObject.h>
#include <utils/collections/IRQCircularBuffer.h>

#include "Canbus.h"

#define NPACKET 3  // equals the number of board

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

struct CanData
{
    uint32_t canId =
        0;  // the id of the can packet without the last 7 bits (sequence bit)
    uint8_t len;
    uint8_t nRec = 0;
    uint64_t payload[32];
} data[NPACKET];

class CanProtocol : public ActiveObject
{
private:
    CanbusDriver* can;
    IRQCircularBuffer<CanData, NPACKET>* buffer;

public:
    CanProtocol(CanbusDriver* can, IRQCircularBuffer<CanData, NPACKET>* buffer)
    {
        this->can    = can;
        this->buffer = buffer;
    }

    void sendCan(CanData toSend)  //@requires toSen to not be empty
    {
        CanPacket packet;
        packet.ext    = true;
        packet.length = toSend.len;
        packet.id     = (toSend.canId << 7) & idMask.firstPacket &
                    (toSend.len & idMask.leftToSend);
        packet.data[0] = toSend.payload[0];
        can->send(packet);
        uint8_t tempLen = toSend.len - 1;
        for (int i = 1; i < toSend.len; i++)
        {
            packet.id =
                packet.id & !idMask.firstPacket & (tempLen & idMask.leftToSend);
            tempLen--;
            packet.data[i] = toSend.payload[i];
        }
    }

    /* Destructor */
    ~CanProtocol() {}

protected:
    void run() override
    {
        uint32_t sourceId;
        CanPacket packet;
        // Infinite loop
        while (true)
        {
            can->getRXBuffer().waitUntilNotEmpty();
            if (!can->getRXBuffer().isEmpty())
            {
                packet   = can->getRXBuffer().pop().packet;
                sourceId = packet.id & idMask.source;

                if (data[sourceId].canId == 0 ||
                    (data[sourceId].canId & idMask.source) == sourceId)
                {
                    if (sourceId & idMask.firstPacket)  // it is a first
                                                        // packet of a data;
                    {
                        data[sourceId].len = packet.id & idMask.leftToSend;
                        data[sourceId].canId =
                            packet.id >> 7;  // discard the sequence number
                                             // data[i].len = packet.length;
                    }
                    if ((data[sourceId].len - data[sourceId].nRec) ==
                        (packet.id & idMask.leftToSend))
                    {
                        uint64_t tempPayload;
                        for (int f = 0; f < packet.length; f++)
                        {
                            tempPayload =
                                tempPayload & (packet.data[f] << (56 - f * 8));
                        }

                        data[sourceId]
                            .payload[data[sourceId].len -
                                     (packet.id & idMask.leftToSend)] =
                            tempPayload;
                        data[sourceId].nRec++;
                        if (data[sourceId].nRec == data[sourceId].len)
                        {
                            buffer->put(data[sourceId]);
                        }
                    }
                    break;
                }
            }
        }
    }
};
}  // namespace Canbus
}  // namespace Boardcore