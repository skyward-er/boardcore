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

#include <Common.h>

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
    uint16_t payload_length;
    uint8_t checksum[2];

    UBXUnpackedFrame() = default;

    UBXUnpackedFrame(uint8_t cls, uint8_t id, const uint8_t* payload,
                     uint16_t payload_length)
        : cls(cls), id(id), payload_length(payload_length)
    {
        memcpy(preamble, UBX_VALID_PREAMBLE, 2);

        memcpy(this->payload, payload,
               std::min(payload_length, UBX_MAX_PAYLOAD_LENGTH));

        calcChecksum(checksum);
    }

    inline uint16_t getFrameLength() const { return payload_length + 8; }

    inline bool isValid() const
    {
        if (memcmp(preamble, UBX_VALID_PREAMBLE, 2) != 0)
        {
            return false;
        }

        if (payload_length > UBX_MAX_PAYLOAD_LENGTH)
        {
            return false;
        }

        uint8_t valid_checksum[2];
        calcChecksum(valid_checksum);
        return memcmp(checksum, valid_checksum, 2) == 0;
    }

    inline void writePacked(uint8_t* frame) const
    {
        memcpy(frame, preamble, 2);
        frame[2] = cls;
        frame[3] = id;
        memcpy(&frame[4], &payload_length, 2);
        memcpy(&frame[6], payload, payload_length);
        memcpy(&frame[6 + payload_length], checksum, 2);
    }

    inline void readPacked(const uint8_t* frame)
    {
        memcpy(preamble, frame, 2);
        cls = frame[2];
        id  = frame[3];
        memcpy(&payload_length, &frame[4], 2);
        memcpy(payload, &frame[6],
               std::min(payload_length, UBX_MAX_PAYLOAD_LENGTH));
        memcpy(checksum, &frame[6 + payload_length], 2);
    }

    inline void calcChecksum(uint8_t* checksum) const
    {
        uint8_t data[UBX_MAX_FRAME_LENGTH];
        data[0] = cls;
        data[1] = id;
        memcpy(&data[2], &payload_length, 2);
        memcpy(&data[4], payload,
               std::min(payload_length, UBX_MAX_PAYLOAD_LENGTH));

        uint16_t data_length =
            std::min(payload_length, UBX_MAX_PAYLOAD_LENGTH) + 4;

        checksum[0] = 0;
        checksum[1] = 0;

        for (uint8_t i = 0; i < data_length; ++i)
        {
            checksum[0] += data[i];
            checksum[1] += checksum[0];
        }
    }
};

}  // namespace Boardcore
