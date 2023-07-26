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

using namespace miosix;

namespace Boardcore
{

namespace Canbus
{

CanProtocol::CanProtocol(CanbusDriver* can, MsgHandler onReceive)
{
    // We assume the bus to be configured at its max velocity
    CanProtocol(can, onReceive, 500 * 1000);
}

CanProtocol::CanProtocol(CanbusDriver* can, MsgHandler onReceive,
                         uint32_t baudRate)
    : can(can), onReceive(onReceive)
{
    loadEstimator = new BusLoadEstimation(baudRate);
}

bool CanProtocol::start()
{
    stopFlag = false;

    if (can == nullptr)
        return false;

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

    return sndStarted && rcvStarted;
}

bool CanProtocol::isStarted() { return sndStarted && rcvStarted; }

void CanProtocol::stop()
{
    stopFlag = true;

    // Wait for sender to stop
    sndThread->join();
}

bool CanProtocol::enqueueMsg(const CanMessage& msg)
{
    // Append the message to the queue
    outQueue.put(msg);

    // Update stats
    // updateQueueStats(appended);

    // Return always true because the circular buffer overrides current packets
    // and can't get full.
    return true;
}

bool CanProtocol::addFilter(uint8_t src, uint64_t dst)
{
    if (src > 0xF || dst > 0xF)
        return false;

    // The filter mask will cover only the source and destination bits
    uint32_t mask = static_cast<uint32_t>(CanProtocolIdMask::SOURCE) |
                    static_cast<uint32_t>(CanProtocolIdMask::DESTINATION);

    uint32_t id =
        src << static_cast<uint8_t>(CanProtocolShiftInformation::SOURCE) |
        dst << static_cast<uint8_t>(CanProtocolShiftInformation::DESTINATION);

    Mask32FilterBank filterBank(id, mask, 1, 1, 0, 0, 0);

    if (can == nullptr)
        return false;
    else
        return can->addFilter(filterBank);
}

void CanProtocol::sendMessage(const CanMessage& msg)
{
    CanPacket packet    = {};
    uint32_t leftToSend = msg.length - 1;

    // Create the id for the first packet
    packet.ext = true;  // Use extended packet id

    // The number of left to send packets
    packet.id = static_cast<uint32_t>(msg.id) |
                ((static_cast<uint32_t>(0x3F) - leftToSend) &
                 static_cast<uint32_t>(CanProtocolIdMask::LEFT_TO_SEND));
    packet.length = byteForUint64(msg.payload[0]);

    // Splits payload[0] in the right number of uint8_t
    for (int i = 0; i < packet.length; i++)
        packet.data[i] = msg.payload[0] >> (8 * i);

    // Send the first packet
    can->send(packet);
    // Updates the loadEstimator
    loadEstimator->addPacket(packet);
    leftToSend--;

    // Prepare the remaining packets
    for (int i = 1; i < msg.length; i++)
    {
        packet.id =
            static_cast<uint32_t>(msg.id) |
            static_cast<uint32_t>(CanProtocolIdMask::FIRST_PACKET_FLAG) |
            ((static_cast<uint32_t>(0x3F) - leftToSend) &
             static_cast<uint32_t>(CanProtocolIdMask::LEFT_TO_SEND));
        packet.length = byteForUint64(msg.payload[i]);

        // Splits payload[i] in the right number of uint8_t
        for (int k = 0; k < packet.length; k++)
            packet.data[k] = msg.payload[i] >> (8 * k);

        can->send(packet);
        // Updates the loadEstimator
        loadEstimator->addPacket(packet);
        leftToSend--;
    }
}

bool CanProtocol::enqueueEvent(uint8_t priority, uint8_t primaryType,
                               uint8_t source, uint8_t destination,
                               uint8_t secondaryType)
{
    return enqueueSimplePacket(priority, primaryType, source, destination,
                               secondaryType, 0xFF);
}

bool CanProtocol::enqueueSimplePacket(uint8_t priority, uint8_t primaryType,
                                      uint8_t source, uint8_t destination,
                                      uint8_t secondaryType, uint64_t payload)
{
    if (priority > 0xF || primaryType > 0x3F || source > 0xF ||
        destination > 0xF || secondaryType > 0xF)
        return false;

    CanMessage msg{};

    // Length set to a minumum of 1 even if there is no payload
    msg.length     = 1;
    msg.payload[0] = payload;

    // clang-format off
    msg.id =  priority      << static_cast<uint32_t>(CanProtocolShiftInformation::PRIORITY);
    msg.id |= primaryType   << static_cast<uint32_t>(CanProtocolShiftInformation::PRIMARY_TYPE);
    msg.id |= source        << static_cast<uint32_t>(CanProtocolShiftInformation::SOURCE);
    msg.id |= destination   << static_cast<uint32_t>(CanProtocolShiftInformation::DESTINATION);
    msg.id |= secondaryType << static_cast<uint32_t>(CanProtocolShiftInformation::SECONDARY_TYPE);
    // clang-format off

    LOG_DEBUG(logger, "Sending message with id: {:x}", msg.id);

    return enqueueMsg(msg);
}

void CanProtocol::runReceiver()
{
    CanMessage msg;
    uint8_t nReceived = 0;

    while (!stopFlag)
    {
        // Wait for the next packet
        can->getRXBuffer().waitUntilNotEmpty();

        // If the buffer is not empty retrieve the packet
        if (!can->getRXBuffer().isEmpty())
        {
            CanPacket pkt = can->getRXBuffer().pop().packet;

            uint8_t leftToReceive =
                static_cast<uint32_t>(0x3F) -
                (pkt.id &
                 static_cast<uint32_t>(CanProtocolIdMask::LEFT_TO_SEND));

            // Check if the packet is the first in the sequence, if this is the
            // case then the previous message is overriden
            if ((pkt.id & static_cast<uint32_t>(
                              CanProtocolIdMask::FIRST_PACKET_FLAG)) == 0)
            {
                // If it is we save the id (without the sequence number) and the
                // message length
                msg.id = pkt.id & static_cast<uint32_t>(
                                      CanProtocolIdMask::MESSAGE_INFORMATION);
                msg.length = leftToReceive + 1;

                // Reset the number of received packets
                nReceived = 0;
            }

            // Accept the packet only if it has the expected id
            // clang-format off
            if (msg.id != -1 &&
                (pkt.id & static_cast<uint32_t>(CanProtocolIdMask::MESSAGE_INFORMATION)) ==
                (msg.id & static_cast<uint32_t>(CanProtocolIdMask::MESSAGE_INFORMATION)))
            // clang-format on
            {
                // Check if the packet is expected in the sequence. The received
                // packet must have the expected left to send value

                if (msg.length - nReceived - 1 == leftToReceive)
                {
                    uint64_t payload = 0;

                    // Assemble the packet data into a uint64_t
                    for (uint8_t i = 0; i < pkt.length; i++)
                    {
                        uint64_t tmp = pkt.data[i];
                        payload |= tmp << (i * 8);
                    }

                    // Add the data to the message
                    msg.payload[msg.length - leftToReceive - 1] = payload;
                    nReceived++;
                }
            }

            // If we have received the right number of packet call onReceive and
            // reset the message
            if (nReceived == msg.length && nReceived != 0)
            {
                LOG_DEBUG(logger, "Message ready with id: {:x}", msg.id);

                onReceive(msg);

                // Reset the packet
                msg.id     = -1;
                msg.length = 0;
            }
        }
    }
}

void CanProtocol::runSender()
{
    LOG_DEBUG(logger, "Sender is running");
    CanMessage msg;

    while (!stopFlag)
    {
        outQueue.waitUntilNotEmpty();

        if (!outQueue.isEmpty())
        {
            // Get the first packet in the queue, without removing it
            msg = outQueue.pop();

            LOG_DEBUG(logger, "Sending message, length: {}", msg.length);

            sendMessage(msg);

            // updateSenderStats();
        }
        else
        {
            // Wait before sending something else
            miosix::Thread::sleep(50);
        }
    }
}

void CanProtocol::rcvLauncher(void* arg)
{
    reinterpret_cast<CanProtocol*>(arg)->runReceiver();
}

void CanProtocol::sndLauncher(void* arg)
{
    reinterpret_cast<CanProtocol*>(arg)->runSender();
}

uint8_t CanProtocol::byteForUint64(uint64_t number)
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

}  // namespace Canbus

}  // namespace Boardcore
