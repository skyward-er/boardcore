/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "UBXUnpackedFrame.h"
#include "UbloxGPSData.h"

namespace Boardcore
{

/**
 * @brief Sensor for Ublox GPS.
 *
 * This sensor handles communication and setup with Ublox GPSs. It uses the
 * binary UBX protocol to retrieve and parse navigation data quicker than using
 * the string based NMEA.
 *
 * At initialization it configures the device with the specified sample rate,
 * resets the configuration and sets up UBX messages and GNSS parameters.
 *
 * Communication with the device is performed through SPI.
 *
 * This driver was written for a NEO-M9N gps with the latest version of UBX.
 */
class UbloxGPS : public Sensor<UbloxGPSData>, public ActiveObject
{
public:
    /**
     * @brief Constructor.
     *
     * @param bus The Spi bus.
     * @param cs The CS pin to lower when we need to sample.
     * @param config The SPI configuration.
     * @param samplerate Sample rate to communicate with the device
     */
    UbloxGPS(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
             SPIBusConfig spiConfig = getDefaultSPIConfig(),
             uint8_t samplerate     = 250);

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns The default SPIBusConfig.
     */
    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief Disables the NMEA messages, configures GNSS options and enables
     * UBX-PVT message
     *
     * @return True if the operation succeeded
     */
    bool init() override;

    /**
     * @brief Reads a single message form the GPS, waits 2 sample cycles.
     *
     * @return True if a message was sampled
     */
    bool selfTest() override;

private:
    UbloxGPSData sampleImpl() override;

    void run() override;

    bool resetConfiguration();

    bool disableNMEAMessages();

    bool setGNSSConfiguration();

    bool enableUBXMessages();

    bool setSampleRate();

    bool parseUBXFrame(const UBXUnpackedFrame& frame);

    bool parseUBXNAVFrame(const UBXUnpackedFrame& frame);

    bool parseUBXACKFrame(const UBXUnpackedFrame& frame);

    bool writeUBXFrame(const UBXUnpackedFrame& frame);

    bool readUBXFrame(UBXUnpackedFrame& frame);

    bool writeRaw(uint8_t* data, size_t size);

    bool readRaw(uint8_t* data, size_t size);

    SPISlave spiSlave;

    const uint8_t samplerate;  // [Hz]

    mutable miosix::FastMutex sampleMutex;
    UbloxGPSData lastSample{};

    PrintLogger logger = Logging::getLogger("ubloxgps");
};

}  // namespace Boardcore
