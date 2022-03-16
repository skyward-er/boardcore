/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Damiano Amatruda
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

#include <algorithm>

namespace Boardcore
{

static constexpr uint16_t UBX_MAX_PAYLOAD_LENGTH = 92;
static constexpr uint16_t UBX_MAX_FRAME_LENGTH   = UBX_MAX_PAYLOAD_LENGTH + 8;
static constexpr uint8_t UBX_VALID_PREAMBLE[]    = {0xb5, 0x62};

struct UBXUnpackedFrame
{
    uint8_t preamble[2];
    uint8_t cls;
    uint8_t id;
    uint8_t payload[UBX_MAX_PAYLOAD_LENGTH];
    uint16_t payloadLength;
    uint8_t checksum[2];

    UBXUnpackedFrame() = default;

    UBXUnpackedFrame(uint8_t cls, uint8_t id, const uint8_t* payload,
                     uint16_t payloadLength)
        : cls(cls), id(id), payloadLength(payloadLength)
    {
        memcpy(preamble, UBX_VALID_PREAMBLE, 2);

        memcpy(this->payload, payload,
               std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH));

        calcChecksum(checksum);
    }

    inline uint16_t getFrameLength() const { return payloadLength + 8; }

    inline bool isValid() const
    {
        if (memcmp(preamble, UBX_VALID_PREAMBLE, 2) != 0)
        {
            return false;
        }

        if (payloadLength > UBX_MAX_PAYLOAD_LENGTH)
        {
            return false;
        }

        uint8_t validChecksum[2];
        calcChecksum(validChecksum);
        return memcmp(checksum, validChecksum, 2) == 0;
    }

    inline void writePacked(uint8_t* frame) const
    {
        memcpy(frame, preamble, 2);
        frame[2] = cls;
        frame[3] = id;
        memcpy(&frame[4], &payloadLength, 2);
        memcpy(&frame[6], payload, payloadLength);
        memcpy(&frame[6 + payloadLength], checksum, 2);
    }

    inline void readPacked(const uint8_t* frame)
    {
        memcpy(preamble, frame, 2);
        cls = frame[2];
        id  = frame[3];
        memcpy(&payloadLength, &frame[4], 2);
        memcpy(payload, &frame[6],
               std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH));
        memcpy(checksum, &frame[6 + payloadLength], 2);
    }

    inline void calcChecksum(uint8_t* checksum) const
    {
        uint8_t data[UBX_MAX_FRAME_LENGTH];
        data[0] = cls;
        data[1] = id;
        memcpy(&data[2], &payloadLength, 2);
        memcpy(&data[4], payload,
               std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH));

        uint16_t dataLength =
            std::min(payloadLength, UBX_MAX_PAYLOAD_LENGTH) + 4;

        checksum[0] = 0;
        checksum[1] = 0;

        for (uint8_t i = 0; i < dataLength; ++i)
        {
            checksum[0] += data[i];
            checksum[1] += checksum[0];
        }
    }
};

}  // namespace Boardcore
