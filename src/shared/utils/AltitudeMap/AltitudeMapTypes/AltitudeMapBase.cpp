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

#include "AltitudeMapBase.h"

#include <algorithm>
#include <cmath>

namespace Boardcore
{

using Units::Length::Meter;

bool AltitudeMapBase::isHeaderValid(const MapHeader& h)
{
    return h.numPointsE != 0 && h.numPointsN != 0 && std::isfinite(h.stepE) &&
           h.stepE > 0 && std::isfinite(h.stepN) && h.stepN > 0 &&
           std::isfinite(h.topleftE) && std::isfinite(h.topleftN) &&
           std::isfinite(h.minAltitude) &&
           std::isfinite(h.maxAltitude - h.minAltitude) &&
           h.maxAltitude >= h.minAltitude;
}

bool AltitudeMapBase::areFinite(Meter n, Meter e)
{
    return std::isfinite(n.value()) && std::isfinite(e.value());
}

bool AltitudeMapBase::init(std::ifstream& file, const MapHeader& mapHeader,
                           std::size_t payloadSize)
{
    if (!isHeaderValid(mapHeader))
    {
        LOG_ERR(logger, "Altitude map file has invalid header values");
        return false;
    }

    header = mapHeader;

    boundaries.eMin = Meter(header.topleftE);
    boundaries.nMax = Meter(header.topleftN);
    boundaries.eMax =
        Meter(header.topleftE + header.stepE * (header.numPointsE - 1));
    boundaries.nMin =
        Meter(header.topleftN - header.stepN * (header.numPointsN - 1));

    if (!std::isfinite(boundaries.eMax.value()) ||
        !std::isfinite(boundaries.nMin.value()))
    {
        LOG_ERR(logger, "Altitude map boundaries are not finite");
        return false;
    }

    return loadPayload(file, payloadSize);
}

bool AltitudeMapBase::isInsideMap(Meter n, Meter e) const
{
    return e.value() >= boundaries.eMin.value() &&
           e.value() <= boundaries.eMax.value() &&
           n.value() >= boundaries.nMin.value() &&
           n.value() <= boundaries.nMax.value();
}

MapBoundaries AltitudeMapBase::getMapBoundaries() const { return boundaries; }

void AltitudeMapBase::toGridCoordinates(Meter n, Meter e, float& row,
                                        float& col) const
{
    const float maxRow = static_cast<float>(header.numPointsN - 1);
    const float maxCol = static_cast<float>(header.numPointsE - 1);

    row = std::max(
        0.0f, std::min((header.topleftN - n.value()) / header.stepN, maxRow));
    col = std::max(
        0.0f, std::min((e.value() - header.topleftE) / header.stepE, maxCol));
}

Meter AltitudeMapBase::getClosestGroundAltitude(Meter n, Meter e) const
{
    if (!areFinite(n, e))
    {
        LOG_ERR(logger, "Invalid coordinates");
        return Meter(0);
    }

    if (!isInsideMap(n, e))
    {
        LOG_WARN(logger,
                 "Point (n:{:.6f} m, e:{:.6f} m) is outside the altitude map, "
                 "using closest point on the map to calculate altitude",
                 n.value(), e.value());
    }

    float row, col;
    toGridCoordinates(n, e, row, col);

    return Meter(altitudeAtGridPosition(row, col));
}

}  // namespace Boardcore
