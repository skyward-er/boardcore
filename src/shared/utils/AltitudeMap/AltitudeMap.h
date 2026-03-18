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

#include "AltitudeMapData.h"

namespace Boardcore
{
/**
 * @brief Class to retrieve altitude data from flash memory.
 *
 * This class provides methods to retrieve altitude data from flash memory if
 * provided with a correctly formatted binary file. It is used by the Flare
 * algorithm determine when to activate. x and y coordinates are NED in the
 * target's frame of reference.
 */
class AltitudeMap
{
public:
    /*
        @param startAddress: the flash memory address where the altitude map is
       stored. The altitude map must be stored in a binary file that begins with
       data structured as MapHeader, followed by a sequence of uint8_t values
       representing the altitude at each point in the map, stored in row-major
       order.
    */
    AltitudeMap(const uint8_t* startAddress);

    /**
     * @brief Initialize the altitude map. Sets the map boundaries and checks
     * the validity of the map header.
     * @return true if initialization was successful, false otherwise.
     */
    bool init();

    /**
     * @brief Check if the given coordinates are inside the map boundaries.
     */
    bool isInsideMap(float x, float y);

    /**
     * @brief Uses the map data to find the closest ground altitude.
     * @return the ground altitude at the given coordinates, or the ground
     * altitude at the closest point on the map if the coordinates are outside
     * the map boundaries. Returns NAN if the map is not initialized.
     */

    float getClosestGroundAltitude(float x, float y);

    /**
     * @brief Get the map boundaries.
     * @return the map boundaries as a MapBoundaries struct. The coordinates are
     * in meters from the target landing point, with the target landing point
     * being roughly at (0, 0). x is positive in the east direction and negative
     * in the west direction, y is positive in the north direction and negative
     * in the south direction.
     */
    MapBoundaries getMapBoundaries();

private:
    const uint8_t* startAddress;
    const MapHeader* header;

    MapBoundaries boundaries;

    float getGroundAltitude(float x, float y);

    bool isInitialized = false;
    PrintLogger logger = Logging::getLogger("AltitudeMap");
    miosix::FastMutex altitudeMapMutex;
};
}  // namespace Boardcore
