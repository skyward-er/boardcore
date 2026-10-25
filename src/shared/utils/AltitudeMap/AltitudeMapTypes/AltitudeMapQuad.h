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
 * @brief Quadtree altitude map format, whoAmI = 0x43.
 *
 * File layout: MapHeader, followed by the tree in depth-first order. Every
 * node starts with one byte:
 *  - 0..254: leaf. The value covers the whole region of the node and maps
 *    linearly to [minAltitude, maxAltitude]:
 *      altitude = minAltitude + v / 254 * (maxAltitude - minAltitude)
 *  - 255: internal node. It is followed by the sizes in bytes of its first
 *    three children (InternalNode<Config::QuadOffset>), then by the four
 *    children in the order top-left, top-right, bottom-left, bottom-right. The
 *    size of the fourth child is implicit.
 *
 * A region [row0, row1] x [col0, col1] is split at
 * midRow = (row0 + row1) / 2 and midCol = (col0 + col1) / 2.
 */
class AltitudeMapQuad final : public AltitudeMapBase
{
private:
    static constexpr uint8_t internalNodeMarker = 255;

    bool loadPayload(std::ifstream& file, std::size_t payloadSize) override;
    float altitudeAtGridPosition(float row, float col) const override;

    /**
     * @brief Walks the tree down to the leaf covering (row, col).
     * @return the raw leaf value, or internalNodeMarker if the tree is
     * malformed.
     */
    uint8_t queryNode(uint16_t row, uint16_t col) const;

    std::vector<uint8_t> quadTreeData;
};

}  // namespace Boardcore
