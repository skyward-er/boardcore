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
 * At initialization it resets the device, sets the configuration and sets up
 * UBX messages and GNSS parameters.
 *
 * Communication with the device is performed through SPI.
 *
 * This sensor was written for a NEO-M9N gps.
 */
class UbloxGPS : public Sensor<UbloxGPSData>
{
public:
    /**
     * @brief UBX frame classes.
     */
    enum UBXFrameClass : uint8_t
    {
        UBX_ACK = 0x05,
        UBX_CFG = 0x06,
        UBX_NAV = 0x01
    };

    /**
     * @brief UBX frame IDs.
     */
    enum UBXFrameID : uint8_t
    {
        UBX_ACK_NAK    = 0x00,
        UBX_ACK_ACK    = 0x01,
        UBX_CFG_RST    = 0x04,
        UBX_CFG_VALSET = 0x8a,
        UBX_NAV_PVT    = 0x07
    };

    /**
     * @brief Payload of UBX frames UBX-ACK-ACK and UBX-ACK-NAK.
     */
    struct __attribute__((packed)) UBXPayloadACK
    {
        uint8_t clsID;  // Class ID of the Acknowledged Message
        uint8_t msgID;  // Message ID of the Acknowledged Message
    };

    /**
     * @brief Payload of UBX frame UBX-NAV-PVT.
     */
    struct __attribute__((packed)) UBXPayloadNAVPVT
    {
        uint32_t iTOW;     // GPS time of week of the navigation epoch [ms]
        uint16_t year;     // Year (UTC) [y]
        uint8_t month;     // Month, range 1..12 (UTC) [month]
        uint8_t day;       // Day of month, range 1..31 (UTC) [d]
        uint8_t hour;      // Hour of day, range 0..23 (UTC) [h]
        uint8_t min;       // Minute of hour, range 0..59 (UTC) [min]
        uint8_t sec;       // Seconds of minute, range 0..60 (UTC) [s]
        uint8_t valid;     // Validity flags
        uint32_t tAcc;     // Time accuracy estimate (UTC) [ns]
        int32_t nano;      // Fraction of second, range -1e9 .. 1e9 (UTC) [ns]
        uint8_t fixType;   // GNSS fix Type
        uint8_t flags;     // Fix status flags
        uint8_t flags2;    // Additional flags
        uint8_t numSV;     // Number of satellites used in Nav Solution
        int32_t lon;       // Longitude {1e-7} [deg]
        int32_t lat;       // Latitude {1e-7} [deg]
        int32_t height;    // Height above ellipsoid [mm]
        int32_t hMSL;      // Height above mean sea level [mm]
        uint32_t hAcc;     // Horizontal accuracy estimate [mm]
        uint32_t vAcc;     // Vertical accuracy estimate [mm]
        int32_t velN;      // NED north velocity [mm/s]
        int32_t velE;      // NED east velocity [mm/s]
        int32_t velD;      // NED down velocity [mm/s]
        int32_t gSpeed;    // Ground Speed (2-D) [mm/s]
        int32_t headMot;   // Heading of motion (2-D) {1e-5} [deg]
        uint32_t sAcc;     // Speed accuracy estimate [mm/s]
        uint32_t headAcc;  // Heading accuracy estimate (both motion and
                           // vehicle) {1e-5} [deg]
        uint16_t pDOP;     // Position DOP {0.01}
        uint16_t flags3;   // Additional flags
        uint8_t reserved0[4];  // Reserved
        int32_t headVeh;       // Heading of vehicle (2-D) {1e-5} [deg]
        int16_t magDec;        // Magnetic declination {1e-2} [deg]
        uint16_t magAcc;       // Magnetic declination accuracy {1e-2} [deg]
    };

    /**
     * @brief Constructor.
     *
     * @param spiBus The SPI bus.
     * @param spiCs The CS pin to lower when we need to sample.
     * @param spiConfig The SPI configuration.
     */
    UbloxGPS(SPIBusInterface& spiBus, miosix::GpioPin spiCs,
             SPIBusConfig spiConfig = getDefaultSPIConfig());

    /**
     * @brief Constructs the default config for the SPI bus.
     *
     * @return The default SPIBusConfig object.
     */
    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool selfTest() override;

private:
    UbloxGPSData sampleImpl() override;

    /**
     * @brief Resets the device.
     *
     * @return True if the device reset succeeded.
     */
    bool reset();

    /**
     * @brief Configures the device.
     *
     * @return True if the device configuration succeeded.
     */
    bool setConfiguration();

    /**
     * @brief Writes a UBX frame and waits for its acknowledgement.
     *
     * @param frame The UBX frame to write.
     * @return True if the frame is valid and acknowledged.
     */
    bool safeWriteUBXFrame(const UBXUnpackedFrame& frame);

    /**
     * @brief Sends a poll request for a UBX frame with the specified class and
     * ID and reads it.
     *
     * @param cls The class of the requested frame.
     * @param id The ID of the requested frame.
     * @return True if the received frame that was requested is valid.
     */
    bool pollReadUBXFrame(uint8_t cls, uint8_t id, UBXUnpackedFrame& frame);

    /**
     * @brief Writes a UBX frame.
     *
     * @param frame The frame to write.
     * @return True if the frame to write is valid.
     */
    bool writeUBXFrame(const UBXUnpackedFrame& frame);

    /**
     * @brief Reads a UBX frame.
     *
     * @param frame The received frame.
     * @return True if the received frame is valid.
     */
    bool readUBXFrame(UBXUnpackedFrame& frame);

    SPISlave spiSlave;

    PrintLogger logger = Logging::getLogger("ubloxgps");
};

}  // namespace Boardcore
