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

AltitudeMap::AltitudeMap(const unsigned char* startAddress)
{
    this->startAddress =
        startAddress;  // Flash memory altitude map start address
    this->header = reinterpret_cast<const MapHeader*>(
        startAddress);  // Altitude map header address
}

bool AltitudeMap::init()
{
    printf("WhoAmI: %x\n", header->whoAmI);
    if (header->whoAmI != 0x42)
    {
        LOG_ERR(logger, "WhoAmI mismatch: expected 0x42, got 0x%02X",
                header->whoAmI);
        return false;
    }

    xMin = header->topleftX;
    yMax = header->topleftY;
    xMax = header->topleftX + header->stepX * (header->numPointsX - 1);
    yMin = header->topleftY - header->stepY * (header->numPointsY - 1);

    isInitialized = true;

    return true;
}

bool AltitudeMap::isInsideMap(float x, float y)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return false;
    }

    return ((x > xMin && x < xMax) && (y > yMin && y < yMax));
}

float AltitudeMap::getGroundAltitude(float x, float y)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return NAN;
    }

    if (!isInsideMap(x, y))
    {
        LOG_ERR(logger, "Point (%f, %f) is outside the altitude map!", x, y);
        return NAN;
    }

    uint16_t indexX = static_cast<uint16_t>(std::round(x / header->stepX));
    uint16_t indexY = static_cast<uint16_t>(std::round(y / header->stepY));

    if (indexX >= header->numPointsX)
        indexX = header->numPointsX - 1;
    if (indexY >= header->numPointsY)
        indexY = header->numPointsY - 1;

    uint16_t altitudeIndex = indexY * header->numPointsX + indexX;

    uint8_t compressedAltitude = *(startAddress + altitudeIndex);

    float groundAltitude = header->minAltitude +
                           (static_cast<float>(compressedAltitude) / 255.0f) *
                               (header->maxAltitude - header->minAltitude);

    return groundAltitude;
}

}  // namespace Boardcore
