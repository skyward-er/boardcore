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
class UbloxGPS : public Sensor<GPSData>, public ActiveObject
{
public:
    UbloxGPS(uint8_t samplerate) : samplerate(samplerate) {}

    /**
     * @brief Disables the NMEA messages, configures GNSS options and enables
     * UBX-PVT message
     *
     * @return True if the operation succeeded
     */
    virtual bool init() override;

    /**
     * @brief Reads a single message form the GPS, waits 2 sample cycles.
     *
     * @return True if a message was sampled
     */
    bool selfTest() override;

protected:
    GPSData sampleImpl() override;

    void run() override;

    bool resetConfiguration();

    virtual bool setupCommunication() { return true; }

    bool disableNMEAMessages();

    bool setGNSSConfiguration();

    bool enableUBXMessages();

    bool setSampleRate();

    bool parseUBXFrame(const UBXUnpackedFrame& frame);

    bool parseUBXNAVFrame(const UBXUnpackedFrame& frame);

    bool parseUBXACKFrame(const UBXUnpackedFrame& frame);

    bool writeUBXFrame(const UBXUnpackedFrame& frame);

    bool readUBXFrame(UBXUnpackedFrame& frame);

    virtual bool writeRaw(uint8_t* data, size_t size) = 0;

    virtual bool readRaw(uint8_t* data, size_t size) = 0;

    const uint8_t samplerate;  // [Hz]

    mutable miosix::FastMutex sample_mutex;
    GPSData last_sample{};

    PrintLogger logger = Logging::getLogger("ubloxgps");
};

class UbloxGPSSPI : public UbloxGPS
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
    UbloxGPSSPI(SPIBusInterface& spi_bus, miosix::GpioPin spi_cs,
                SPIBusConfig spi_config = getDefaultSPIConfig(),
                uint8_t samplerate      = 250);

    /**
     * Constructs the default config for SPI Bus.
     *
     * @returns The default SPIBusConfig.
     */
    static SPIBusConfig getDefaultSPIConfig();

private:
    virtual bool writeRaw(uint8_t* data, size_t size) override;

    virtual bool readRaw(uint8_t* data, size_t size) override;

    SPISlave spi_slave;
};

class UbloxGPSSerial : public UbloxGPS
{
public:
    /**
     * @brief Serial constructor.
     *
     * @param serial_port_number Number of the serial port
     * @param serial_port_name Name of the file for the gps device
     * @param serial_baudrate Baudrate to communicate with the device (max:
     * 921600, min: 4800 for NEO-M9N)
     * @param samplerate GPS sample rate (max: 25 for NEO-M9N)
     * @param serial_default_baudrate Startup baudrate (38400 for NEO-M9N)
     */
    UbloxGPSSerial(int serial_port_number       = 2,
                   const char* serial_port_name = "gps",
                   int serial_baudrate          = 921600,
                   int serial_default_baudrate  = 38400,
                   uint8_t samplerate           = 10);

private:
    /**
     * @brief Sets up the serial port with the correct baudrate
     *
     * Opens the serial port with the default baudrate and changes it to
     * the value specified in the constructor, then it reopens the serial port.
     * If the device is already using the correct baudrate this won't have
     * effect. However if the gps is using a different baudrate the diver won't
     * be able to communicate.
     */
    virtual bool setupCommunication() override;

    bool setBaudrate();

    virtual bool writeRaw(uint8_t* data, size_t size) override;

    virtual bool readRaw(uint8_t* data, size_t size) override;

    const int serial_port_number;
    const char* serial_port_name;
    const int serial_baudrate;          // [baud]
    const int serial_default_baudrate;  // [baud]
    char serial_file_path[16];  // Allows for a filename of up to 10 characters
    int serial_file = -1;
};

}  // namespace Boardcore
