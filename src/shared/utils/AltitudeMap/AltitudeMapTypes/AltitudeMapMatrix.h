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

#pragma once

#include <cstdint>
#include <vector>

#include "AltitudeMapBase.h"

namespace Boardcore
{

/**
 * @brief Matrix (flat grid) altitude map format, whoAmI = 0x42.
 *
 * File layout: MapHeader, followed by numPointsN * numPointsE uint8_t values in
 * row-major order. Raw value v in [0, 255] maps linearly to
 * [minAltitude, maxAltitude]:
 *
 *   altitude = minAltitude + v / 255 * (maxAltitude - minAltitude)
 *
 * Altitudes between grid points are bilinearly interpolated.
 */
class AltitudeMapMatrix final : public AltitudeMapBase
{
private:
    bool loadPayload(std::ifstream& file, std::size_t payloadSize) override;

    /**
     * @brief Bilinear interpolation between the four grid points surrounding
     * (row, col).
     */
    float altitudeAtGridPosition(float row, float col) const override;

    /**
     * @brief Decoded altitude of one grid point.
     * @return 0 if row or col are out of range.
     */
    float cellAltitude(uint16_t row, uint16_t col) const;

    std::vector<uint8_t> mapData;
};

}  // namespace Boardcore
