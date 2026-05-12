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
    this->header = reinterpret_cast<const MapHeader*>(
        startAddress);  // Altitude map header address
    this->mapData =
        startAddress +
        sizeof(MapHeader);  // Flash memory altitude map data start address
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

    boundaries.eMin = Meter(header->topleftE);
    boundaries.nMax = Meter(header->topleftN);
    boundaries.eMax =
        Meter(header->topleftE + header->stepE * (header->numPointsE - 1));
    boundaries.nMin =
        Meter(header->topleftN - header->stepN * (header->numPointsN - 1));

    isInitialized = true;

    return true;
}

bool AltitudeMap::isInsideMap(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return false;
    }

    return ((e >= boundaries.eMin && e <= boundaries.eMax) &&
            (n >= boundaries.nMin && n <= boundaries.nMax));
}

MapBoundaries AltitudeMap::getMapBoundaries()
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return MapBoundaries();
    }

    return boundaries;
}

Meter AltitudeMap::getAltitudeAtIndex(uint16_t indexN, uint16_t indexE)
{
    if (indexE >= header->numPointsE)
        indexE = header->numPointsE - 1;
    if (indexN >= header->numPointsN)
        indexN = header->numPointsN - 1;

    uint32_t altitudeIndex     = indexN * header->numPointsE + indexE;
    uint8_t compressedAltitude = *(mapData + altitudeIndex);

    float altitude = header->minAltitude +
                     (static_cast<float>(compressedAltitude) / 255.0f) *
                         (header->maxAltitude - header->minAltitude);

    return Meter(altitude);
}

Meter AltitudeMap::getGroundAltitude(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return Meter(NAN);
    }

    if (!isInsideMap(n, e))
        LOG_ERR(logger,
                "Point (n:{:.6f} m, e:{:.6f} m) is outside the altitude map!",
                n.value(), e.value());

    uint16_t indexE = static_cast<uint16_t>(
        std::round((e.value() - header->topleftE) / header->stepE));
    uint16_t indexN = static_cast<uint16_t>(
        std::round((header->topleftN - n.value()) / header->stepN));

    return getAltitudeAtIndex(indexN, indexE);
    ;
}

Meter AltitudeMap::getClosestGroundAltitude(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
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

Meter AltitudeMap::getInterpolatedGroundAltitude(Meter n, Meter e)
{
    if (!isInitialized)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return Meter(NAN);
    }

    if (!isInsideMap(n, e))
        return getClosestGroundAltitude(n, e);

    float fe = (e.value() - header->topleftE) / header->stepE;
    float fn = (header->topleftN - n.value()) / header->stepN;

    // TopLeft of interpolation square
    uint16_t e0 = static_cast<uint16_t>(std::floor(fe));
    uint16_t n0 = static_cast<uint16_t>(std::floor(fn));

    // BottomRight of interpolation square
    uint16_t e1 = e0 + 1;
    uint16_t n1 = n0 + 1;

    float de = fe - e0;
    float dn = fn - n0;

    if (e0 >= header->numPointsE - 1)
    {
        e0 = e1 = header->numPointsE - 1;
        de      = 0.0f;
    }
    if (n0 >= header->numPointsN - 1)
    {
        n0 = n1 = header->numPointsN - 1;
        dn      = 0.0f;
    }

    Meter z00 = getAltitudeAtIndex(n0, e0);
    Meter z10 = getAltitudeAtIndex(n0, e1);
    Meter z01 = getAltitudeAtIndex(n1, e0);
    Meter z11 = getAltitudeAtIndex(n1, e1);

    Meter z0 = z00 * (1.0f - de) + z10 * de;
    Meter z1 = z01 * (1.0f - de) + z11 * de;

    Meter interpolatedAltitude = z0 * (1.0f - dn) + z1 * dn;

    return interpolatedAltitude;
}

}  // namespace Boardcore
