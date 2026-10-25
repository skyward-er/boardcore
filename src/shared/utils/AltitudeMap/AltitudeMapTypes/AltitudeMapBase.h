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

#include <diagnostic/PrintLogger.h>
#include <units/Length.h>

#include <cstddef>
#include <cstdint>
#include <fstream>

#include "../AltitudeMapData.h"

namespace Boardcore
{

/**
 * @brief Common base of all the altitude map file formats.
 *
 * Only required methods are:
 *  - loadPayload(): read its own payload from the file;
 *  - altitudeAtIndex(): return the altitude of one grid cell.
 *
 * n and e coordinates are NED in the target's frame of reference. Grid cell
 * (row 0, col 0) is the top-left corner (maximum n, minimum e) of the map.
 *
 * Use AltitudeMap class (not this) in entrypoints.
 */
class AltitudeMapBase
{
public:
    virtual ~AltitudeMapBase() = default;

    /**
     * @brief Validates the header, computes the boundaries and loads the
     * payload.
     * @param file An ifstream of the altitude map file at its beginning.
     * @param mapHeader The header already read from the file.
     * @param payloadSize Number of bytes between the header and the end of the
     * file.
     * @return true if initialization was successful.
     */
    bool init(std::ifstream& file, const MapHeader& mapHeader,
              std::size_t payloadSize);

    /**
     * @brief Check if the given coordinates are inside the map boundaries.
     */
    bool isInsideMap(Units::Length::Meter n, Units::Length::Meter e) const;

    /**
     * @brief Get the map boundaries.
     */
    MapBoundaries getMapBoundaries() const;

    /**
     * @brief Altitude of the grid cell nearest to the given coordinates. If the
     * coordinates are outside the map, the nearest point on the map is used.
     * @return the altitude, or 0 for a corrupt payload.
     */
    Units::Length::Meter getClosestGroundAltitude(Units::Length::Meter n,
                                                  Units::Length::Meter e) const;

protected:
    /**
     * @brief Reads the format-specific payload. The stream is positioned at the
     * first payload byte.
     */
    virtual bool loadPayload(std::ifstream& file, std::size_t payloadSize) = 0;

    /**
     * @brief Altitude at a fractional grid position. Integer values of row and
     * col are the grid points themselves.
     * @return the altitude in meters, or 0 if the payload is corrupt.
     */
    virtual float altitudeAtGridPosition(float row, float col) const = 0;

    MapHeader header{};
    MapBoundaries boundaries;
    mutable PrintLogger logger = Logging::getLogger("AltitudeMap");

private:
    void toGridCoordinates(Units::Length::Meter n, Units::Length::Meter e,
                           float& row, float& col) const;

    static bool isHeaderValid(const MapHeader& h);
    static bool areFinite(Units::Length::Meter n, Units::Length::Meter e);
};

}  // namespace Boardcore
