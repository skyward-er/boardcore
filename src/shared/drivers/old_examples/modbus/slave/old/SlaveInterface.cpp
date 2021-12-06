/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Silvano Seva
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

#include "SlaveInterface.h"

using namespace std;
using namespace miosix;

SlaveInterface::SlaveInterface(miosix::STM32Serial* phy, miosix::gpioPin* rxEn,
                               uint8_t addr)
    : phy(phy), rxEn(rxEn), addr(addr)
{

    if (addr == 0 || addr > 247)
    {
#ifndef __NO_EXCEPTIONS
        throw invalid_argument("Slave address out of range!");
#else
        puts("Slave address out of range!");
#endif
    }

    rxEn->mode(Mode::OUTPUT);
    rxEn->low();
}

void SlaveInterface::receive()
{

    /* The packet is structured as follows:
     * --------------------------------------------------
     * | slave addr | func code | data len | data | CRC |
     * --------------------------------------------------
     */

    uint8_t head[3] = {0};  // first of all get the header
    if (phy->readBlock(head, 3, 0) != 0)
    {

        size_t size               = head[2];
        uint8_t payload[size + 2] = {0};  // payload size + 2 bytes of CRC
        phy->readBlock(payload, size + 2, 0);

        /* Here we have the first important check: is the packet addressed to
         * this slave? The check is done here in order to flush away the rx
         * buffer in case the packet is not for this slave
         */
        if (head[0] != addr)
        {
            return;  // not for this slave, return
        }

        /* Since CRC is calculated on data len + data we have to pack all the
         * bytes in a suitable chunk on which calculate the CRC
         */
        uint8_t chunk[size + 1] = {0};
        chunk[0]                = size;
        memcpy(&chunk[1], payload, size);

        // The CRC of the received packet is stored into the last two bytes of
        // the payload buffer, indexed at size and size + 1
        uint16_t pktCrc = *(reinterpret_cast<uint16_t*>(&payload[size]));
        if (pktCrc == CRC16(chunk, size + 1))
        {
            received.reset(PDU(head[1], payload, size));
        }
    }
}

unique_ptr<PDU> SlaveInterface::getPacket() { return move(received); }

void SlaveInterface::sendReply(unique_ptr<PDU> reply)
{

    auto payload   = reply->data();
    size_t pktSize = payload.first + 4;

    uint8_t packet[pktSize] = {0};
    packet[0]               = reply->funcCode();
    packet[1]               = payload.first;  // length of the data field

    std::memcpy(&packet[2], payload.second, payload.first);
    uint16_t crc        = CRC16(&packet[1], payload.first + 1);
    packet[pktSize - 2] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    packet[pktSize - 1] = static_cast<uint8_t>(crc & 0xFF);

    rxEn->high();
    phy->writeBlock(packet, pktSize, 0);
    rxEn->low();
}

uint16_t SlaveInterface::CRC16(uint8_t* data, size_t len)
{

    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {

        crc ^= static_cast<uint16_t>(data[i]);
        for (int j = 0; j < 8; j++)
        {

            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
