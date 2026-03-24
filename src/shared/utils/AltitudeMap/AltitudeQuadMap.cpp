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
#include <cstring>
#include <fstream>
#include <iostream>

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
    isInitialized = false;

    std::ifstream file(mapFilename, std::ios::binary | std::ios::ate);

    if (!file)
    {
        std::cout << "Failed to open altitude quadtree map file: "
                  << mapFilename << std::endl;
        LOG_ERR(logger, "Failed to open altitude quadtree map file: {}",
                mapFilename);
        return false;
    }

    const auto endPosition = file.tellg();
    if (endPosition == std::streampos(-1))
        return false;

    auto size = static_cast<std::size_t>(endPosition);

    if (size <= sizeof(MapHeader))
    {
        std::cout << "Quadtree map file size is smaller than map header size"
                  << std::endl;
        LOG_ERR(logger,
                "Quadtree map file size is smaller than map header size");
        return false;
    }

    auto treeSize = size - sizeof(MapHeader);

    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(&header), sizeof(header)))
        return false;

    if (header.whoAmI != 0x43)
    {
        LOG_ERR(logger, "WhoAmI mismatch: expected 0x43, got 0x{:02X}",
                header.whoAmI);
        return false;
    }

    if (header.numPointsE == 0 || header.numPointsN == 0 ||
        !std::isfinite(header.stepE) || header.stepE <= 0 ||
        !std::isfinite(header.stepN) || header.stepN <= 0 ||
        !std::isfinite(header.topleftE) || !std::isfinite(header.topleftN) ||
        !std::isfinite(header.minAltitude) ||
        !std::isfinite(header.maxAltitude - header.minAltitude) ||
        header.maxAltitude < header.minAltitude)
        return false;

    quadTreeData.resize(treeSize);
    if (!file.read(reinterpret_cast<char*>(quadTreeData.data()), treeSize))
        return false;

    quadTreeSize = treeSize;

    boundaries.eMin = Meter(header.topleftE);
    boundaries.nMax = Meter(header.topleftN);
    boundaries.eMax =
        Meter(header.topleftE + header.stepE * (header.numPointsE - 1));
    boundaries.nMin =
        Meter(header.topleftN - header.stepN * (header.numPointsN - 1));

    if (!std::isfinite(boundaries.eMax.value()) ||
        !std::isfinite(boundaries.nMin.value()))
        return false;

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
    std::size_t offset = 0;

    uint16_t row0 = 0;
    uint16_t row1 = header.numPointsN - 1;

    uint16_t col0 = 0;
    uint16_t col1 = header.numPointsE - 1;

    while (offset < quadTreeSize)
    {
        const uint8_t value = quadTreeData[offset];

        if (value != 255)  // 255 marks an internal node.
            return value;

        if (quadTreeSize - offset < sizeof(Node))
            break;

        Node node{};
        std::memcpy(&node, quadTreeData.data() + offset, sizeof(node));

        const std::size_t child0 = offset + sizeof(Node);
        if (node.size0 == 0 || node.size0 > quadTreeSize - child0)
            break;
        const std::size_t child1 = child0 + node.size0;
        if (node.size1 == 0 || node.size1 > quadTreeSize - child1)
            break;
        const std::size_t child2 = child1 + node.size1;
        if (node.size2 == 0 || node.size2 >= quadTreeSize - child2)
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

    return 255;  // Invalid tree; this value cannot represent an altitude.
}

Meter AltitudeQuadMap::getGroundAltitude(Meter n, Meter e)
{
    // Clamp before converting to an integer, including at rounded boundaries.
    const float col =
        std::max(0.0f, std::min((e.value() - header.topleftE) / header.stepE,
                                static_cast<float>(header.numPointsE - 1)));
    const float row =
        std::max(0.0f, std::min((header.topleftN - n.value()) / header.stepN,
                                static_cast<float>(header.numPointsN - 1)));

    const uint8_t compressed =
        queryNode(static_cast<uint16_t>(row), static_cast<uint16_t>(col));

    if (compressed == 255)
        return Meter(NAN);

    const float altitude =
        header.minAltitude +
        (compressed / 254.0f) * (header.maxAltitude - header.minAltitude);

    return Meter(altitude);
}

Meter AltitudeQuadMap::getClosestGroundAltitude(Meter n, Meter e)
{
    if (!std::isfinite(n.value()) || !std::isfinite(e.value()))
        return Meter(NAN);

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
