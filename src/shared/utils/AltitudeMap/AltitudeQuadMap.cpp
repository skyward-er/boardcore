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

#include "AltitudeQuadMap.h"

#include <cmath>
#include <fstream>

namespace Boardcore
{

using namespace Units::Length;

using Offset = Config::QuadOffset;
using Node   = InternalNode<Offset>;

AltitudeQuadMap::AltitudeQuadMap(const char* mapFilename)
    : mapFilename(mapFilename)
{
}

bool AltitudeQuadMap::init()
{
    std::ifstream file(mapFilename, std::ios::binary | std::ios::ate);

    if (!file)
    {
        LOG_ERR(logger, "Failed to open altitude quadtree map file: {}",
                mapFilename);
        return false;
    }

    auto size = static_cast<std::size_t>(file.tellg());

    if (size < sizeof(MapHeader))
    {
        LOG_ERR(logger,
                "Quadtree map file size is smaller than map header size");
        return false;
    }

    auto treeSize = size - sizeof(MapHeader);

    if (treeSize > quadTreeData.size())
    {
        LOG_ERR(logger, "Configured quadtree map buffer too small");
        return false;
    }

    file.seekg(0);

    file.read(reinterpret_cast<char*>(&header), sizeof(header));

    if (header.whoAmI != 0x43)
    {
        LOG_ERR(logger, "WhoAmI mismatch: expected 0x43, got 0x{:02X}",
                header.whoAmI);
        return false;
    }

    file.read(reinterpret_cast<char*>(quadTreeData.data()), treeSize);

    quadTreeSize = treeSize;

    boundaries.eMin = Meter(header.topleftE);
    boundaries.nMax = Meter(header.topleftN);
    boundaries.eMax =
        Meter(header.topleftE + header.stepE * (header.numPointsE - 1));
    boundaries.nMin =
        Meter(header.topleftN - header.stepN * (header.numPointsN - 1));

    isInitialized = true;

    return true;
}

bool AltitudeQuadMap::isInsideMap(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeQuadMap not initialized!");
        return false;
    }

    return ((e >= boundaries.eMin && e <= boundaries.eMax) &&
            (n >= boundaries.nMin && n <= boundaries.nMax));
}

MapBoundaries AltitudeQuadMap::getMapBoundaries()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeQuadMap not initialized!");
        return MapBoundaries();
    }

    return boundaries;
}

uint8_t AltitudeQuadMap::queryNode(uint16_t row, uint16_t col) const
{
    uint32_t offset = 0;

    uint16_t row0 = 0;
    uint16_t row1 = header.numPointsN - 1;

    uint16_t col0 = 0;
    uint16_t col1 = header.numPointsE - 1;

    while (true)
    {
        const uint8_t value = quadTreeData[offset];

        if (value != 255)  // Leaf node marker is 255
            return value;

        const auto* node =
            reinterpret_cast<const Node*>(quadTreeData.data() + offset);

        const uint32_t child0 = offset + sizeof(Node);

        const uint32_t child1 = child0 + node->size0;

        const uint32_t child2 = child1 + node->size1;

        const uint32_t child3 = child2 + node->size2;

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
}

Meter AltitudeQuadMap::getGroundAltitude(Meter n, Meter e)
{
    int col = static_cast<int>(
        std::floor((e.value() - header.topleftE) / header.stepE));

    int row = static_cast<int>(
        std::floor((header.topleftN - n.value()) / header.stepN));

    col = std::max(0, std::min(col, static_cast<int>(header.numPointsE) - 1));
    row = std::max(0, std::min(row, static_cast<int>(header.numPointsN) - 1));

    const uint8_t compressed =
        queryNode(static_cast<uint16_t>(row), static_cast<uint16_t>(col));

    const float altitude =
        header.minAltitude +
        (compressed / 254.0f) * (header.maxAltitude - header.minAltitude);

    return Meter(altitude);
}

Meter AltitudeQuadMap::getClosestGroundAltitude(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeQuadMap not initialized!");
        return Meter(NAN);
    }

    if (!isInsideMap(n, e))
    {
        LOG_WARN(logger,
                 "Point (n:{:.6f} m, e:{:.6f} m) is outside the altitude map, "
                 "using closest "
                 "point on the map to calculate altitude",
                 n.value(), e.value());
    }

    Meter closestE = std::max(boundaries.eMin, std::min(boundaries.eMax, e));
    Meter closestN = std::max(boundaries.nMin, std::min(boundaries.nMax, n));

    return getGroundAltitude(closestN, closestE);
}

}  // namespace Boardcore
