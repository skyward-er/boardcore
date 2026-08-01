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

#include "AltitudeQuadMapData.h"

namespace Boardcore
{
/**
 * @brief Class to retrieve altitude data from flash memory.
 *
 * This class provides methods to retrieve altitude data from flash memory if
 * provided with a correctly formatted binary file. It is used by the Flare
 * algorithm determine when to activate. n and e coordinates are NED in the
 * target's frame of reference.
 */
class AltitudeQuadMap
{
public:
    /**
     * @param mapFilename: the path to the binary file containing the altitude
     * map. The altitude map must be stored in a binary file that begins with
     * data structured as MapHeader, followed by an array of uint8_t values
     * representing the quadtree-compressed altitude data. The altitude values
     * are compressed to a range of 0-254, where 0 corresponds to the minimum
     * altitude and 254 corresponds to the maximum altitude defined in the
     * MapHeader. A value of 255 is reserved to indicate an internal node in the
     * quadtree structure.
     */
    explicit AltitudeQuadMap(const char* mapFilename);
    /**
     * @brief Initialize the altitude map. Sets the map boundaries and checks
     * the validity of the map header.
     * @return true if initialization was successful, false otherwise.
     */
    bool init();

    /**
     * @brief Check if the given coordinates are inside the map boundaries.
     */
    bool isInsideMap(Units::Length::Meter n, Units::Length::Meter e);

    /**
     * @brief Uses the map data to find the closest ground altitude.
     * @return the ground altitude at the given coordinates, or the ground
     * altitude at the closest point on the map if the coordinates are outside
     * the map boundaries. Returns NAN if the map is not initialized.
     */

    Units::Length::Meter getClosestGroundAltitude(Units::Length::Meter n,
                                                  Units::Length::Meter e);

    /**
     * @brief Get the map boundaries.
     * @return the map boundaries as a MapBoundaries struct. The coordinates are
     * in meters from the target landing point, with the target landing point
     * being roughly at (0, 0). n is positive in the north direction and
     * negative in the south direction, e is positive in the east direction and
     * negative in the west direction.
     */
    MapBoundaries getMapBoundaries();

private:
    const char* mapFilename;

    std::array<uint8_t, Config::MAX_QUADTREE_SIZE> quadTreeData;
    std::size_t quadTreeSize = 0;

    MapHeader header;
    MapBoundaries boundaries;

    Units::Length::Meter getGroundAltitude(Units::Length::Meter n,
                                           Units::Length::Meter e);

    uint8_t queryNode(uint16_t row, uint16_t col) const;

    bool isInitialized = false;
    PrintLogger logger = Logging::getLogger("AltitudeMap");
    miosix::FastMutex altitudeMapMutex;
};
}  // namespace Boardcore
