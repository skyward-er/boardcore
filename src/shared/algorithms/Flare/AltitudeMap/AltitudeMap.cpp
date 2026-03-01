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

#include "AltitudeMap.h"

#include <cmath>

namespace Boardcore
{

AltitudeMap::AltitudeMap(const uint8_t* startAddress)
{
    this->startAddress =
        startAddress;  // Flash memory altitude map start address
    this->header = reinterpret_cast<const MapHeader*>(
        startAddress);  // Altitude map header address
    this->tiles = reinterpret_cast<const TileDescriptor*>(
        startAddress + sizeof(MapHeader));  // Address pointing to the start of
                                            // the tile descriptors
}

bool AltitudeMap::isInsideMap(float x, float y) const
{
    return ((x > header->xMin && x < header->xMax) &&
            (y > header->yMin && y < header->yMax));
}

const TileDescriptor* AltitudeMap::getTileDescriptor(float x, float y) const
{
    for (uint16_t i = 0; i < header->numTilesX * header->numTilesY; i++)
    {
        const TileDescriptor* tile = &tiles[i];

        float bottomRightX =
            tile->topleftX + tile->stepX * (tile->numPointsX - 1);
        float bottomRightY =
            tile->topleftY + tile->stepY * (tile->numPointsY - 1);

        if ((x >= tile->topleftX && x <= bottomRightX) &&
            (y <= tile->topleftY && y >= bottomRightY))
            return tile;
    }
    return nullptr;
}

float AltitudeMap::getGroundAltitude(float x, float y) const
{
    if (!isInsideMap(x, y))
        return NAN;

    const TileDescriptor* tile = getTileDescriptor(x, y);

    if (tile == nullptr)
        return NAN;

    float localX = x - tile->topleftX;
    float localY = y - tile->topleftY;

    uint16_t indexX = static_cast<uint16_t>(std::round(localX / tile->stepX));
    uint16_t indexY = static_cast<uint16_t>(std::round(localX / tile->stepX));

    if (indexX >= tile->numPointsX)
        indexX = tile->numPointsX - 1;
    if (indexY >= tile->numPointsY)
        indexY = tile->numPointsY - 1;

    uint16_t altitudeIndex = indexY * tile->numPointsX + indexX;

    uint8_t compressedAltitude =
        *(startAddress + tile->dataOffset + altitudeIndex);

    float groundAltitude = tile->tileMinAltitude +
                           (static_cast<float>(compressedAltitude) / 255.0f) *
                               (tile->tileMaxAltitude - tile->tileMinAltitude);

    return groundAltitude;
}

}  // namespace Boardcore
