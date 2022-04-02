/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio, Damiano Amatruda
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
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "UBXFrame.h"
#include "UBXGPSData.h"

namespace Boardcore
{

/**
 * @brief Sensor for u-blox GPS.
 *
 * This sensor handles communication and setup with u-blox GPSs. It uses the
 * binary UBX protocol to retrieve and parse navigation data faster than using
 * the string-based NMEA.
 *
 * At initialization it resets the device, sets the configuration and sets up
 * UBX messages and GNSS parameters.
 *
 * Communication with the device is performed through SPI.
 *
 * This sensor is compatible with series NEO-M8 and NEO-M9.
 */
class UBXGPS : public Sensor<UBXGPSData>
{
public:
    /**
     * @brief Constructor.
     *
     * @param spiBus The SPI bus.
     * @param spiCs The CS pin to lower when we need to sample.
     * @param spiConfig The SPI configuration.
     * @param rate The GPS sample rate.
     */
    UBXGPS(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
           SPIBusConfig spiConfig = getDefaultSPIConfig(), uint8_t rate = 5);

    /**
     * @brief Constructs the default config for the SPI bus.
     *
     * @return The default SPIBusConfig object.
     */
    static SPIBusConfig getDefaultSPIConfig();

    uint8_t getRate();

    bool init() override;

    bool selfTest() override;

private:
    UBXGPSData sampleImpl() override;

    /**
     * @brief Resets the device to its default configuration.
     *
     * @return True if the device reset succeeded.
     */
    bool reset();

    /**
     * @brief Disables NMEA messages on the SPI port.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool disableNMEAMessages();

    /**
     * @brief Configures the dynamic model to airborn 4g.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setDynamicModelToAirborne4g();

    /**
     * @brief Configures the navigation solution update rate.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setUpdateRate();

    /**
     * @brief Configures the PVT message output rate.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setPVTMessageRate();

    /**
     * @brief Reads a UBX frame.
     *
     * @param frame The received frame.
     * @return True if a valid frame was read.
     */
    bool readUBXFrame(UBXFrame& frame);

    /**
     * @brief Writes a UBX frame.
     *
     * @param frame The frame to write.
     * @return True if the frame is valid.
     */
    bool writeUBXFrame(const UBXFrame& frame);

    /**
     * @brief Writes a UBX frame and waits for its acknowledgement.
     *
     * @param frame The frame to write.
     * @return True if the frame is valid and acknowledged.
     */
    bool safeWriteUBXFrame(const UBXFrame& frame);

    /**
     * @brief Sends a poll request for a UBX frame for the specified message and
     * reads it.
     *
     * Note that for some messages a payload is required even for polling (e.g.
     * UBX-CFG-PRT)
     *
     * @param message Requested message identifier.
     * @param frame Frame filled with data.
     * @return True if the received frame that was requested is valid.
     */
    bool pollReadUBXFrame(UBXMessage message, UBXFrame& frame);

    SPISlave spiSlave;
    uint8_t rate;

    PrintLogger logger = Logging::getLogger("ubxgps");
};

}  // namespace Boardcore
