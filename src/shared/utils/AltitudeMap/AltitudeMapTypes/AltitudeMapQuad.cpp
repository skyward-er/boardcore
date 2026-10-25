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

#include "AltitudeMapQuad.h"

#include <cmath>
#include <cstring>

namespace Boardcore
{

using Offset = Config::QuadOffset;
using Node   = InternalNode<Offset>;

bool AltitudeMapQuad::loadPayload(std::ifstream& file, std::size_t payloadSize)
{
    quadTreeData.resize(payloadSize);
    if (!file.read(reinterpret_cast<char*>(quadTreeData.data()),
                   static_cast<std::streamsize>(payloadSize)))
    {
        LOG_ERR(logger, "Failed to read quadtree data from file");
        quadTreeData.clear();
        return false;
    }

    return true;
}

uint8_t AltitudeMapQuad::queryNode(uint16_t row, uint16_t col) const
{
    const std::size_t treeSize = quadTreeData.size();
    std::size_t offset         = 0;

    uint16_t row0 = 0;
    uint16_t row1 = header.numPointsN - 1;

    uint16_t col0 = 0;
    uint16_t col1 = header.numPointsE - 1;

    while (offset < treeSize)
    {
        const uint8_t value = quadTreeData[offset];

        if (value != internalNodeMarker)
            return value;

        if (treeSize - offset < sizeof(Node))
            break;

        Node node{};
        std::memcpy(&node, quadTreeData.data() + offset, sizeof(node));

        const std::size_t child0 = offset + sizeof(Node);
        if (node.size0 == 0 || node.size0 > treeSize - child0)
            break;
        const std::size_t child1 = child0 + node.size0;
        if (node.size1 == 0 || node.size1 > treeSize - child1)
            break;
        const std::size_t child2 = child1 + node.size1;
        if (node.size2 == 0 || node.size2 >= treeSize - child2)
            break;
        const std::size_t child3 = child2 + node.size2;

        const uint16_t midRow = (row0 + row1) >> 1;
        const uint16_t midCol = (col0 + col1) >> 1;

        if (row <= midRow)
        {
            if (col <= midCol)
            {
                offset = child0;

                row1 = midRow;
                col1 = midCol;
            }
            else
            {
                offset = child1;

                row1 = midRow;
                col0 = midCol + 1;
            }
        }
        else if (col <= midCol)
        {
            offset = child2;

            row0 = midRow + 1;
            col1 = midCol;
        }
        else
        {
            offset = child3;

            row0 = midRow + 1;
            col0 = midCol + 1;
        }
    }

    return internalNodeMarker;  // Invalid tree; cannot represent an altitude.
}

float AltitudeMapQuad::altitudeAtGridPosition(float row, float col) const
{
    const uint8_t compressed =
        queryNode(static_cast<uint16_t>(std::round(row)),
                  static_cast<uint16_t>(std::round(col)));

    if (compressed == internalNodeMarker)
        return 0;

    return header.minAltitude + (static_cast<float>(compressed) / 254.0f) *
                                    (header.maxAltitude - header.minAltitude);
}

}  // namespace Boardcore
