/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Damiano Amatruda, Alberto Nidasio
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

#include <algorithm>
#include <cstring>

namespace Boardcore
{

/**
 * @brief UBX messages enumeration.
 */
enum class UBXMessage : uint16_t
{
    UBX_NAV_PVT  = 0x0107,  // Navigation position velocity time solution
    UBX_ACK_NAK  = 0x0500,  // Message acknowledged
    UBX_ACK_ACK  = 0x0501,  // Message not acknowledged
    UBX_CFG_PRT  = 0x0600,  // Port configuration
    UBX_CFG_MSG  = 0x0601,  // Set message rate
    UBX_CFG_RST  = 0x0604,  // Reset receiver
    UBX_CFG_RATE = 0x0608,  // Navigation/measurement rate settings
    UBX_CFG_NAV5 = 0x0624,  // Navigation engine settings
};

struct UBXFrame
{
    static constexpr uint16_t UBX_MAX_PAYLOAD_LENGTH = 92;
    static constexpr uint16_t UBX_MAX_FRAME_LENGTH = UBX_MAX_PAYLOAD_LENGTH + 8;
    static constexpr uint8_t UBX_PREAMBLE[]        = {0xb5, 0x62};

    uint8_t preamble[2];
    uint16_t message;
    uint16_t payloadLength;
    uint8_t payload[UBX_MAX_PAYLOAD_LENGTH];
    uint8_t checksum[2];

    UBXFrame() = default;

    /**
     * @brief Construct a new UBXFrame with the specified message and given
     * payload.
     */
    UBXFrame(UBXMessage message, const uint8_t* payload,
             uint16_t payloadLength);

    /**
     * @brief Return the total frame length.
     */
    uint16_t getLength() const;

    UBXMessage getMessage() const;

    /**
     * @brief Tells whether the current frame is valid or not. Checks the
     * preamble and the checksum.
     */
    bool isValid() const;

    /**
     * @brief Writes the current message into the given array.
     *
     * @param frame Must be an array of the proper length.
     */
    void writePacked(uint8_t* frame) const;

    /**
     * @brief Reads a raw frame.
     *
     * Note that the frame bytes are assumed to start with the preamble.
     *
     * @param frame An array of length at least UBX_MAX_FRAME_LENGTH.
     */
    void readPacked(const uint8_t* frame);

    /**
     * @brief Computes the frame checksum.
     *
     * @param checksum Array of 2 elements where to store the checksum
     */
    void calcChecksum(uint8_t* checksum) const;
};

struct UBXAckFrame : public UBXFrame
{
    // TODO: Do something better!
    // static constexpr size_t UBX_MAX_PAYLOAD_LENGTH = 2;
    // static constexpr size_t UBX_MAX_FRAME_LENGTH   = UBX_MAX_PAYLOAD_LENGTH +
    // 8;

    UBXMessage getAckMessage();

    /**
     * @brief Tells whether the frame is an ack.
     */
    bool isAck();

    /**
     * @brief Tells whether the frame is a nak.
     */
    bool isNack();

    /**
     * @brief Tells whether the frame is an ack frame.
     *
     */
    bool isValid();
};

inline UBXFrame::UBXFrame(UBXMessage message, const uint8_t* payload,
                          uint16_t payloadLength)
    : message(static_cast<uint16_t>(message)), payloadLength(payloadLength)
{
    memcpy(preamble, UBX_PREAMBLE, 2);
    if (payload != nullptr)
        memcpy(this->payload, payload,
               std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH));
    calcChecksum(checksum);
}

inline uint16_t UBXFrame::getLength() const { return payloadLength + 8; }

inline UBXMessage UBXFrame::getMessage() const
{
    return static_cast<UBXMessage>(message);
}

inline bool UBXFrame::isValid() const
{
    if (payloadLength > UBX_MAX_PAYLOAD_LENGTH)
        return false;

    uint8_t validChecksum[2];
    calcChecksum(validChecksum);
    return memcmp(checksum, validChecksum, 2) == 0;
}

inline void UBXFrame::writePacked(uint8_t* frame) const
{
    memcpy(frame, preamble, 2);
    frame[2] = message >> 8;
    frame[3] = message;
    memcpy(&frame[4], &payloadLength, 2);
    memcpy(&frame[6], payload, payloadLength);
    memcpy(&frame[6 + payloadLength], checksum, 2);
}

inline void UBXFrame::readPacked(const uint8_t* frame)
{
    memcpy(preamble, frame, 2);
    message = frame[2] << 8;
    message |= frame[3];
    memcpy(&payloadLength, &frame[4], 2);
    payloadLength = std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH);
    memcpy(payload, &frame[6], payloadLength);
    memcpy(checksum, &frame[6 + payloadLength], 2);
}

inline void UBXFrame::calcChecksum(uint8_t* checksum) const
{
    uint8_t data[UBX_MAX_FRAME_LENGTH];
    data[0] = message >> 8;
    data[1] = message;
    memcpy(&data[2], &payloadLength, 2);
    memcpy(&data[4], payload, std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH));

    uint16_t dataLength = std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH) + 4;

    checksum[0] = 0;
    checksum[1] = 0;

    for (uint8_t i = 0; i < dataLength; ++i)
    {
        checksum[0] += data[i];
        checksum[1] += checksum[0];
    }
}

inline UBXMessage UBXAckFrame::getAckMessage()
{
    return static_cast<UBXMessage>((uint16_t)payload[0] << 8 | payload[1]);
}

inline bool UBXAckFrame::isAck()
{
    return getMessage() == UBXMessage::UBX_ACK_ACK;
}

inline bool UBXAckFrame::isNack()
{
    return getMessage() == UBXMessage::UBX_ACK_NAK;
}

inline bool UBXAckFrame::isValid()
{
    return UBXFrame::isValid() && (isAck() || isNack());
}

}  // namespace Boardcore
