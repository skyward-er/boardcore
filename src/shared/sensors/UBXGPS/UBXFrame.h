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

static constexpr uint16_t UBX_MAX_PAYLOAD_LENGTH = 92;
static constexpr uint16_t UBX_MAX_FRAME_LENGTH   = UBX_MAX_PAYLOAD_LENGTH + 8;
static constexpr uint8_t UBX_PREAMBLE[]          = {0xb5, 0x62};
static constexpr uint8_t UBX_WAIT                = 0xff;

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

/**
 * @brief Generic UBX frame.
 */
struct UBXFrame
{
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

    /**
     * @brief Return the stored payload length.
     */
    uint16_t getRealPayloadLength() const;

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

/**
 * @brief UBX frames UBX-ACK-ACK and UBX-ACK-NAK.
 */
struct UBXAckFrame : public UBXFrame
{
    /**
     * @brief Payload of UBX frames UBX-ACK-ACK and UBX-ACK-NAK.
     */
    struct __attribute__((packed)) Payload
    {
        uint16_t ackMessage;
    };

    Payload& getPayload() const;

    UBXMessage getAckMessage() const;

    /**
     * @brief Tells whether the frame is an ack.
     */
    bool isAck() const;

    /**
     * @brief Tells whether the frame is a nak.
     */
    bool isNack() const;

    /**
     * @brief Tells whether the frame is an ack frame.
     */
    bool isValid() const;
};

/**
 * @brief UBX frame UBX-NAV-PVT.
 */
struct UBXPvtFrame : public UBXFrame
{
public:
    /**
     * @brief Payload of UBX frame UBX-NAV-PVT.
     */
    struct __attribute__((packed)) Payload
    {
        uint32_t iTOW;     // GPS time of week of the navigation epoch [ms]
        uint16_t year;     // Year (UTC) [y]
        uint8_t month;     // Month, range 1..12 (UTC) [month]
        uint8_t day;       // Day of month, range 1..31 (UTC) [d]
        uint8_t hour;      // Hour of day, range 0..23 (UTC) [h]
        uint8_t min;       // Minute of hour, range 0..59 (UTC) [min]
        uint8_t sec;       // Seconds of minute, range 0..60 (UTC) [s]
        uint8_t valid;     // Validity flags
        uint32_t tAcc;     // Time accuracy estimate (UTC) [ns]
        int32_t nano;      // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
        uint8_t fixType;   // GNSS fix Type
        uint8_t flags;     // Fix status flags
        uint8_t flags2;    // Additional flags
        uint8_t numSV;     // Number of satellites used in Nav Solution
        int32_t lon;       // Longitude {1e-7} [deg]
        int32_t lat;       // Latitude {1e-7} [deg]
        int32_t height;    // Height above ellipsoid [mm]
        int32_t hMSL;      // Height above mean sea level [mm]
        uint32_t hAcc;     // Horizontal accuracy estimate [mm]
        uint32_t vAcc;     // Vertical accuracy estimate [mm]
        int32_t velN;      // NED north velocity [mm/s]
        int32_t velE;      // NED east velocity [mm/s]
        int32_t velD;      // NED down velocity [mm/s]
        int32_t gSpeed;    // Ground Speed (2-D) [mm/s]
        int32_t headMot;   // Heading of motion (2-D) {1e-5} [deg]
        uint32_t sAcc;     // Speed accuracy estimate [mm/s]
        uint32_t headAcc;  // Heading accuracy estimate (both motion and
                           // vehicle) {1e-5} [deg]
        uint16_t pDOP;     // Position DOP {0.01}
        uint16_t flags3;   // Additional flags
        uint8_t reserved0[4];  // Reserved
        int32_t headVeh;       // Heading of vehicle (2-D) {1e-5} [deg]
        int16_t magDec;        // Magnetic declination {1e-2} [deg]
        uint16_t magAcc;       // Magnetic declination accuracy {1e-2} [deg]
    };

    Payload& getPayload() const;

    /**
     * @brief Tells whether the frame is an ack frame.
     */
    bool isValid() const;
};

inline UBXFrame::UBXFrame(UBXMessage message, const uint8_t* payload,
                          uint16_t payloadLength)
    : message(static_cast<uint16_t>(message)), payloadLength(payloadLength)
{
    memcpy(preamble, UBX_PREAMBLE, 2);
    if (payload != nullptr)
        memcpy(this->payload, payload, getRealPayloadLength());
    calcChecksum(checksum);
}

inline uint16_t UBXFrame::getLength() const { return payloadLength + 8; }

inline uint16_t UBXFrame::getRealPayloadLength() const
{
    return std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH);
}

inline UBXMessage UBXFrame::getMessage() const
{
    return static_cast<UBXMessage>(message);
}

inline bool UBXFrame::isValid() const
{
    if (memcmp(preamble, UBX_PREAMBLE, 2) != 0)
        return false;

    if (payloadLength > UBX_MAX_PAYLOAD_LENGTH)
        return false;

    uint8_t validChecksum[2];
    calcChecksum(validChecksum);
    return memcmp(checksum, validChecksum, 2) == 0;
}

inline void UBXFrame::writePacked(uint8_t* frame) const
{
    memcpy(frame, preamble, 2);
    memcpy(&frame[2], &message, 2);
    memcpy(&frame[4], &payloadLength, 2);
    memcpy(&frame[6], payload, getRealPayloadLength());
    memcpy(&frame[6 + payloadLength], checksum, 2);
}

inline void UBXFrame::readPacked(const uint8_t* frame)
{
    memcpy(preamble, frame, 2);
    memcpy(&message, &frame[2], 2);
    memcpy(&payloadLength, &frame[4], 2);
    memcpy(payload, &frame[6], getRealPayloadLength());
    memcpy(checksum, &frame[6 + payloadLength], 2);
}

inline void UBXFrame::calcChecksum(uint8_t* checksum) const
{
    uint8_t data[getRealPayloadLength() + 4];
    memcpy(data, &message, 2);
    memcpy(&data[2], &payloadLength, 2);
    memcpy(&data[4], payload, getRealPayloadLength());

    checksum[0] = 0;
    checksum[1] = 0;

    for (size_t i = 0; i < sizeof(data); ++i)
    {
        checksum[0] += data[i];
        checksum[1] += checksum[0];
    }
}

inline UBXAckFrame::Payload& UBXAckFrame::getPayload() const
{
    return (Payload&)payload;
}

inline UBXMessage UBXAckFrame::getAckMessage() const
{
    return static_cast<UBXMessage>(getPayload().ackMessage);
}

inline bool UBXAckFrame::isAck() const
{
    return getMessage() == UBXMessage::UBX_ACK_ACK;
}

inline bool UBXAckFrame::isNack() const
{
    return getMessage() == UBXMessage::UBX_ACK_NAK;
}

inline bool UBXAckFrame::isValid() const
{
    return UBXFrame::isValid() && (isAck() || isNack());
}

inline UBXPvtFrame::Payload& UBXPvtFrame::getPayload() const
{
    return (Payload&)payload;
}

inline bool UBXPvtFrame::isValid() const
{
    return UBXFrame::isValid() && getMessage() == UBXMessage::UBX_NAV_PVT;
}

}  // namespace Boardcore
