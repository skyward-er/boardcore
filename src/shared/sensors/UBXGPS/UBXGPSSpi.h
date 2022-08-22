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
 * @brief Sensor for UBlox GPS.
 *
 * This sensor handles communication and setup with UBlox GPSs. It uses the
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
class UBXGPSSpi : public Sensor<UBXGPSData>
{
public:
    /**
     * @brief Constructor.
     *
     * @param spiBus The SPI bus.
     * @param spiCs The CS pin to lower when we need to sample.
     * @param spiConfig The SPI configuration.
     * @param sampleRate The GPS sample rate [kHz].
     */
    UBXGPSSpi(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
              SPIBusConfig spiConfig = getDefaultSPIConfig(),
              uint8_t sampleRate     = 5);

    /**
     * @brief Constructs the default config for the SPI bus.
     *
     * @return The default SPIBusConfig object.
     */
    static SPIBusConfig getDefaultSPIConfig();

    uint8_t getSampleRate();

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
     * @brief Enables UBX and disables NMEA on the SPI port.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setUBXProtocol();

    /**
     * @brief Configures the dynamic model to airborn 4g.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setDynamicModelToAirborne4g();

    /**
     * @brief Configures the navigation solution sample rate.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setSampleRate();

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

    SPISlave spiSlave;
    uint8_t sampleRate;

    PrintLogger logger = Logging::getLogger("ubxgps");

    static constexpr float MS_TO_TICK = miosix::TICK_FREQ / 1000.f;

    static constexpr unsigned int RESET_SLEEP_TIME = 5000;  // [ms]
    static constexpr unsigned int READ_TIMEOUT     = 500;   // [ms]
    static constexpr unsigned int MAX_TRIES        = 5;     // [1]
};

}  // namespace Boardcore
