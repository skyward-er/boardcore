
#pragma once

#include <diagnostic/PrintLogger.h>
#include <units/Length.h>

#include <memory>
#include <string>

#include "AltitudeMapData.h"
#include "AltitudeMapTypes/AltitudeMapBase.h"

namespace Boardcore
{

/**
 * @brief Class to retrieve altitude data from a map file.
 *
 * Selects the correct implementation based on the map file header. n and e
 * coordinates are NED in the target's frame of reference.
 */
class AltitudeMap
{
public:
    /**
     * @param mapFilename Path to the map file.
     */
    explicit AltitudeMap(const char* mapFilename);

    /**
     * @brief Opens the file, selects the implementation from the header and
     * loads the map.
     * @return true if initialization was successful.
     */
    bool init();

    bool isInitialized() const;

    /**
     * @brief Check if the given coordinates are inside the map boundaries.
     * @return false if they're outside the map or the map is not initialized.
     */
    bool isInsideMap(Units::Length::Meter n, Units::Length::Meter e) const;

    /**
     * @return the map boundaries, all 0 if the map is not initialized.
     */
    MapBoundaries getMapBoundaries() const;

    /**
     * @brief Method to get the altitude of the closest ground point to the
     * given coordinates.
     * @return the altitude, or 0 if the map is not initialized, the
     * coordinates are not finite or the file content is corrupt.
     */
    Units::Length::Meter getClosestGroundAltitude(Units::Length::Meter n,
                                                  Units::Length::Meter e) const;

private:
    std::string mapFilename;

    std::unique_ptr<AltitudeMapBase> impl;

    mutable PrintLogger logger = Logging::getLogger("AltitudeMap");
};

}  // namespace Boardcore
