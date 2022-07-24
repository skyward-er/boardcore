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

#include <stdint.h>

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
 * - Priority           4 bit \
 * - Primary type       6 bit |
 * - Source             4 bit | 22 bits - Message informations
 * - Destination        4 bit |
 * - Secondary type     4 bit /
 * - First packet flag  1 bit \ 7 bits - Sequential informations
 * - Remaining packets  6 bit /
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
 * The CanProtocol allows to transmit arbitrarily sized messages over the CanBus
 * overcoming the 8 byte limitation of each single packet.
 *
 * Our CanProtocol uses the extended can packet, the 29 bits id is divided such
 * as:
 * - Priority           4 bit \
 * - Primary type       6 bit |
 * - Source             4 bit | 22 bits - Message informations
 * - Destination        4 bit |
 * - Secondary type     4 bit /
 * - First packet flag  1 bit \ 7 bits - Sequential informations
 * - Remaining packets  6 bit /
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
enum class CanProtocolIdMask : uint32_t
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

enum CanProtocolShiftInformation : uint8_t
{
    // Shift values for message informations
    PRIORITY       = 25,
    PRIMARY_TYPE   = 19,
    SOURCE         = 15,
    DESTINATION    = 11,
    SECONDARY_TYPE = 7,

    // Shift values for sequential informations
    FIRST_PACKET_FLAG = 6,
    LEFT_TO_SEND      = 0,

    // Position of the message infos relative to the entire can packet id
    SEQUENTIAL_INFORMATION = 7
};

/**
 * @brief Generic struct that contains a can protocol message.
 *
 * For example an accelerometer message could have:
 * - 4 bytes for the timestamp
 * - 3x4 bytes for float values
 * This message would be divided into 2 can packets.
 *
 * Note that the maximum size for a message is 520 bytes since the remaining
 * packet information is 6 bit wide.
 */
struct CanMessage
{
    int32_t id     = -1;  ///< Id of the message without sequential infos.
    uint8_t length = 0;   ///< Length of the message content.
    uint64_t payload[65];

    uint8_t getPriority() const
    {
        return id >>
               static_cast<uint8_t>(CanProtocolShiftInformation::PRIORITY);
    }

    uint8_t getPrimaryType() const
    {
        return (id | static_cast<uint32_t>(CanProtocolIdMask::PRIMARY_TYPE)) >>
               static_cast<uint8_t>(CanProtocolShiftInformation::PRIMARY_TYPE);
    }

    uint8_t getSource() const
    {
        return (id | static_cast<uint32_t>(CanProtocolIdMask::SOURCE)) >>
               static_cast<uint8_t>(CanProtocolShiftInformation::SOURCE);
    }

    uint8_t getDestination() const
    {
        return (id | static_cast<uint32_t>(CanProtocolIdMask::DESTINATION)) >>
               static_cast<uint8_t>(CanProtocolShiftInformation::DESTINATION);
    }

    uint8_t getSecondaryType() const
    {
        return (id |
                static_cast<uint32_t>(CanProtocolIdMask::SECONDARY_TYPE)) >>
               static_cast<uint8_t>(CanProtocolShiftInformation::PRIMARY_TYPE);
    }
};

inline bool operator==(const CanMessage& lhs, const CanMessage& rhs)
{
    if (lhs.id != rhs.id || lhs.length != rhs.length)
        return false;

    for (int i = 0; i < lhs.length; i++)
        if (lhs.payload[i] != rhs.payload[i])
            return false;

    return true;
}

inline bool operator!=(const CanMessage& lhs, const CanMessage& rhs)
{
    return !(lhs == rhs);
}

}  // namespace Canbus

}  // namespace Boardcore
