/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Leonardo Montecchi
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

#include "AltitudeMapMatrix.h"

#include <cmath>

namespace Boardcore
{

bool AltitudeMapMatrix::loadPayload(std::ifstream& file,
                                    std::size_t payloadSize)
{
    const uint32_t expectedPayloadSize =
        static_cast<uint32_t>(header.numPointsN) *
        static_cast<uint32_t>(header.numPointsE);

    if (payloadSize != expectedPayloadSize)
    {
        LOG_ERR(logger, "Altitude map payload size mismatch");
        return false;
    }

    mapData.resize(payloadSize);
    if (!file.read(reinterpret_cast<char*>(mapData.data()),
                   static_cast<std::streamsize>(payloadSize)))
    {
        LOG_ERR(logger, "Failed to read altitude map data from file");
        mapData.clear();
        return false;
    }

    return true;
}

float AltitudeMapMatrix::cellAltitude(uint16_t row, uint16_t col) const
{
    if (row >= header.numPointsN || col >= header.numPointsE)
        return 0;

    const uint32_t index = static_cast<uint32_t>(row) * header.numPointsE + col;

    return header.minAltitude + (static_cast<float>(mapData[index]) / 255.0f) *
                                    (header.maxAltitude - header.minAltitude);
}

float AltitudeMapMatrix::altitudeAtGridPosition(float row, float col) const
{
    const float row0f = std::floor(row);
    const float col0f = std::floor(col);

    // Top-left corner of the interpolation square.
    const uint16_t r0 = static_cast<uint16_t>(row0f);
    const uint16_t c0 = static_cast<uint16_t>(col0f);

    // Bottom-right corner of the interpolation square.
    const uint16_t r1 =
        (r0 + 1 < header.numPointsN) ? static_cast<uint16_t>(r0 + 1) : r0;
    const uint16_t c1 =
        (c0 + 1 < header.numPointsE) ? static_cast<uint16_t>(c0 + 1) : c0;

    const float dr = row - row0f;
    const float dc = col - col0f;

    const float z00 = cellAltitude(r0, c0);
    const float z01 = cellAltitude(r0, c1);
    const float z10 = cellAltitude(r1, c0);
    const float z11 = cellAltitude(r1, c1);

    const float top    = z00 * (1.0f - dc) + z01 * dc;
    const float bottom = z10 * (1.0f - dc) + z11 * dc;

    return top * (1.0f - dr) + bottom * dr;
}

}  // namespace Boardcore
