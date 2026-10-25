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
#include <cstdint>
#include <fstream>

#include "AltitudeMapTypes/AltitudeMapMatrix.h"
#include "AltitudeMapTypes/AltitudeMapQuad.h"

namespace Boardcore
{

using Units::Length::Meter;

AltitudeMap::AltitudeMap(const char* mapFilename)
    : mapFilename(mapFilename != nullptr ? mapFilename : "")
{
}

bool AltitudeMap::init()
{
    impl.reset();

    if (mapFilename.empty())
    {
        LOG_ERR(logger, "Altitude map filename not set");
        return false;
    }

    std::ifstream file(mapFilename.c_str(), std::ios::binary | std::ios::ate);
    if (!file)
    {
        LOG_ERR(logger, "Failed to open altitude map file: {}",
                mapFilename.c_str());
        return false;
    }

    const auto endPosition = file.tellg();
    if (endPosition == std::streampos(-1))
    {
        LOG_ERR(logger, "Failed to get altitude map file size");
        return false;
    }

    const auto fileSize = static_cast<std::size_t>(endPosition);
    if (fileSize <= sizeof(MapHeader))
    {
        LOG_ERR(logger, "Altitude map file size is smaller than map header");
        return false;
    }

    MapHeader header{};
    file.seekg(0);
    if (!file.read(reinterpret_cast<char*>(&header), sizeof(header)))
    {
        LOG_ERR(logger, "Failed to read map header from file: {}",
                mapFilename.c_str());
        return false;
    }

    switch (header.whoAmI)
    {
        case static_cast<uint8_t>(MapFormat::Matrix):
            impl = std::make_unique<AltitudeMapMatrix>();
            break;

        case static_cast<uint8_t>(MapFormat::QuadTree):
            impl = std::make_unique<AltitudeMapQuad>();
            break;

        default:
            LOG_ERR(logger,
                    "WhoAmI is not recognized: expected 0x42(matrix map) or "
                    "0x43(quadtree map), got 0x{:02X}",
                    header.whoAmI);
            return false;
    }

    if (!impl->init(file, header, fileSize - sizeof(MapHeader)))
    {
        impl.reset();
        return false;
    }

    return true;
}

bool AltitudeMap::isInitialized() const { return impl != nullptr; }

bool AltitudeMap::isInsideMap(Meter n, Meter e) const
{
    if (!impl)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return false;
    }

    return impl->isInsideMap(n, e);
}

MapBoundaries AltitudeMap::getMapBoundaries() const
{
    if (!impl)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return MapBoundaries{};
    }

    return impl->getMapBoundaries();
}

Meter AltitudeMap::getClosestGroundAltitude(Meter n, Meter e) const
{
    if (!impl)
    {
        LOG_ERR(logger, "AltitudeMap not initialized!");
        return Meter(0);
    }

    return impl->getClosestGroundAltitude(n, e);
}

}  // namespace Boardcore
