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

#include "CanProtocol.h"

namespace Boardcore
{

namespace Canbus
{

CanData data[N_PACKET];

CanProtocol::CanProtocol(CanbusDriver* can) { this->can = can; }

CanProtocol::~CanProtocol() { (*can).~CanbusDriver(); }

CanData CanProtocol::getPacket()
{
    miosix::Lock<miosix::FastMutex> l(mutex);

    if (!buffer.isEmpty())
        return buffer.pop();
    else
        return {};
}

bool CanProtocol::isBufferEmpty()
{
    miosix::Lock<miosix::FastMutex> l(mutex);
    return buffer.isEmpty();
}

void CanProtocol::waitBufferEmpty() { buffer.waitUntilNotEmpty(); }

void CanProtocol::sendData(CanData dataToSend)
{
    CanPacket packet = {};
    uint8_t tempLen  = dataToSend.length - 1;
    uint32_t tempId  = dataToSend.canId;

    // Send the first packet
    packet.ext = true;
    packet.id  = (tempId << shiftSequentialInfo) | firstPacket |
                (63 - (tempLen & leftToSend));
    packet.length = byteForInt(dataToSend.payload[0]);
    for (int k = 0; k < packet.length; k++)
        packet.data[k] = dataToSend.payload[0] >> (8 * k);
    can->send(packet);
    tempLen--;

    for (int i = 1; i < dataToSend.length; i++)
    {
        tempId = dataToSend.canId;
        packet.id =
            (tempId << shiftSequentialInfo) | (63 - (tempLen & leftToSend));
        packet.length = byteForInt(dataToSend.payload[i]);

        for (int k = 0; k < packet.length; k++)
            packet.data[k] = dataToSend.payload[i] >> (8 * k);

        can->send(packet);
        tempLen--;
    }
}

uint8_t CanProtocol::byteForInt(uint64_t number)
{
    uint8_t i;

    for (i = 1; i <= 8; i++)
    {
        number >>= 8;
        if (number == 0)
            return i;
    }

    return i;
}

void CanProtocol::run()
{
    while (!shouldStop())
    {
        // Wait for the next packet
        can->getRXBuffer().waitUntilNotEmpty();

        // If the buffer is not empty retrieve the packet
        if (!can->getRXBuffer().isEmpty())
        {
            CanPacket packet = can->getRXBuffer().pop().packet;

            // Discard the sequence number
            uint32_t idNoSeq = packet.id >> shiftSequentialInfo;
            uint8_t sourceId = (idNoSeq & source) >> shiftSource;

            // Check for maximum size
            if (sourceId < N_PACKET)
            {
                if (data[sourceId].canId == -1 ||
                    ((data[sourceId].canId & source) >> shiftSource) ==
                        sourceId)
                {
                    uint8_t left = 63 - (packet.id & leftToSend);

                    // Check if it is the first packet in the sequence
                    if ((packet.id & firstPacket) >> shiftFirstPacket)
                    {
                        data[sourceId].length = left + 1;
                        data[sourceId].canId  = idNoSeq;
                    }

                    if ((data[sourceId].length - (data[sourceId].nRec + 1)) ==
                        left)
                    {
                        uint64_t tempPayload = 0;

                        for (int f = 0; f < packet.length; f++)
                        {
                            uint64_t tempData = packet.data[f];
                            tempPayload = tempPayload | (tempData << (f * 8));
                        }

                        if (data[sourceId].length - left - 1 >= 0 &&
                            data[sourceId].length - left - 1 <
                                32)  // check for index
                        {
                            data[sourceId]
                                .payload[data[sourceId].length - left - 1] =
                                tempPayload;
                            data[sourceId].nRec++;
                        }
                    }

                    if (data[sourceId].nRec == data[sourceId].length &&
                        data[sourceId].nRec != 0)
                    {
                        miosix::Lock<miosix::FastMutex> l(mutex);
                        buffer.put(data[sourceId]);

                        // Empties the struct
                        data[sourceId].canId  = -1;
                        data[sourceId].nRec   = 0;
                        data[sourceId].length = 0;
                    }
                }
            }
        }
    }
}

}  // namespace Canbus

}  // namespace Boardcore
