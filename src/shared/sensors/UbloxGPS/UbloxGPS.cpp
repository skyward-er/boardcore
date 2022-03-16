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

#include "UbloxGPS.h"

#include <diagnostic/StackLogger.h>
#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

using namespace miosix;

UbloxGPS::UbloxGPS(SPIBusInterface& spiBus, GpioPin spiCs,
                   SPIBusConfig spiConfig, uint8_t samplerate)
    : spiSlave(spiBus, spiCs, spiConfig), samplerate(samplerate)
{
}

SPIBusConfig UbloxGPS::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_32;
    spiConfig.mode         = SPI::Mode::MODE_1;
    return spiConfig;
}

bool UbloxGPS::init()
{
    // Reset configuration to default
    if (!resetConfiguration())
    {
        return false;
    }

    Thread::sleep(100);

    // Disable NMEA messages
    if (!disableNMEAMessages())
    {
        return false;
    }

    Thread::sleep(100);

    // Set GNSS configuration
    if (!setGNSSConfiguration())
    {
        return false;
    }

    Thread::sleep(100);

    // Enable UBX messages
    if (!enableUBXMessages())
    {
        return false;
    }

    Thread::sleep(100);

    // Set sample rate
    if (!setSampleRate())
    {
        return false;
    }

    return true;
}

bool UbloxGPS::selfTest() { return true; }

UbloxGPSData UbloxGPS::sampleImpl()
{
    Lock<FastMutex> l(sampleMutex);
    return lastSample;
}

void UbloxGPS::run()
{
    UBXUnpackedFrame frame;

    while (!shouldStop())
    {
        StackLogger::getInstance().updateStack(THID_GPS);

        // Try to read the message
        if (!readUBXFrame(frame))
        {
            LOG_DEBUG(logger, "Unable to read a UBX message");
            continue;
        }

        // Parse the message
        if (!parseUBXFrame(frame))
        {
            LOG_DEBUG(
                logger,
                "UBX message not recognized (class: {:#02x}, id: {:#02x})",
                frame.cls, frame.id);
        }
    }
}

bool UbloxGPS::resetConfiguration()
{
    static constexpr uint16_t payloadLength = 4;

    uint8_t payload[payloadLength] = {
        0x00, 0x00,  // navBbrMask (Hot start)
        0x00,        // Hardware reset immediately
        0x00         // Reserved
    };

    UBXUnpackedFrame frame{0x06, 0x04,  // Message UBX-CFG-RST
                           payload, payloadLength};

    return writeUBXFrame(frame);
}

bool UbloxGPS::disableNMEAMessages()
{
    static constexpr uint16_t payloadLength = 34;

    uint8_t payload[payloadLength] = {
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0xbb, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_GGA_UART1 key ID
        0x00,                    // CFG-MSGOUT-NMEA_ID_GGA_UART1 value
        0xca, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_GLL_UART1 key ID
        0x00,                    // CFG-MSGOUT-NMEA_ID_GLL_UART1 value
        0xc0, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_GSA_UART1 key ID
        0x00,                    // CFG-MSGOUT-NMEA_ID_GSA_UART1 value
        0xc5, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_GSV_UART1 key ID
        0x00,                    // CFG-MSGOUT-NMEA_ID_GSV_UART1 value
        0xac, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_RMC_UART1 key ID
        0x00,                    // CFG-MSGOUT-NMEA_ID_RMC_UART1 value
        0xb1, 0x00, 0x91, 0x20,  // CFG-MSGOUT-NMEA_ID_VTG_UART1 key ID
        0x00                     // CFG-MSGOUT-NMEA_ID_VTG_UART1 value
    };

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payloadLength};

    return writeUBXFrame(frame);
}

bool UbloxGPS::setGNSSConfiguration()
{
    static constexpr uint16_t payloadLength = 9;

    uint8_t payload[payloadLength] = {
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x21, 0x00, 0x11, 0x20,  // CFG-NAVSPG-DYNMODEL key ID
        0x08                     // CFG-NAVSPG-DYNMODEL value
    };

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payloadLength};

    return writeUBXFrame(frame);
}

bool UbloxGPS::enableUBXMessages()
{
    static constexpr uint16_t payloadLength = 9;

    uint8_t payload[payloadLength] = {
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x07, 0x00, 0x91, 0x20,  // CFG-MSGOUT-UBX_NAV_PVT_UART1 key ID
        0x01                     // CFG-MSGOUT-UBX_NAV_PVT_UART1 value
    };

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payloadLength};

    return writeUBXFrame(frame);
}

bool UbloxGPS::setSampleRate()
{
    static constexpr uint16_t payloadLength = 10;

    uint8_t payload[payloadLength] = {
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x21, 0x30,  // CFG-RATE-MEAS key ID
        0xff, 0xff               // CFG-RATE-MEAS value (placeholder)
    };
    memcpy(&payload[8], &samplerate, 2);

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payloadLength};

    return writeUBXFrame(frame);
}

bool UbloxGPS::parseUBXFrame(const UBXUnpackedFrame& frame)
{
    switch (frame.cls)  // Message class
    {
        case 0x01:  // UBX-NAV
            return parseUBXNAVFrame(frame);
        case 0x05:  // UBX-ACK
            return parseUBXACKFrame(frame);
    }
    return false;
}

bool UbloxGPS::parseUBXNAVFrame(const UBXUnpackedFrame& frame)
{
    switch (frame.id)  // Message ID
    {
        case 0x07:  // UBX-NAV-PVT
            // Lock the lastSample variable
            Lock<FastMutex> l(sampleMutex);

            // Latitude
            int32_t rawLatitude = frame.payload[28] | frame.payload[29] << 8 |
                                  frame.payload[30] << 16 |
                                  frame.payload[31] << 24;
            lastSample.latitude = (float)rawLatitude / 1e7;

            // Longitude
            int32_t rawLongitude = frame.payload[24] | frame.payload[25] << 8 |
                                   frame.payload[26] << 16 |
                                   frame.payload[27] << 24;
            lastSample.longitude = (float)rawLongitude / 1e7;

            // Height
            int32_t rawHeight = frame.payload[32] | frame.payload[33] << 8 |
                                frame.payload[34] << 16 |
                                frame.payload[35] << 24;
            lastSample.height = (float)rawHeight / 1e3;

            // Velocity north
            int32_t rawVelocityNorth =
                frame.payload[48] | frame.payload[49] << 8 |
                frame.payload[50] << 16 | frame.payload[51] << 24;
            lastSample.velocityNorth = (float)rawVelocityNorth / 1e3;

            // Velocity east
            int32_t rawVelocityEast =
                frame.payload[52] | frame.payload[53] << 8 |
                frame.payload[54] << 16 | frame.payload[55] << 24;
            lastSample.velocityEast = (float)rawVelocityEast / 1e3;

            // Velocity down
            int32_t rawVelocityDown =
                frame.payload[56] | frame.payload[57] << 8 |
                frame.payload[58] << 16 | frame.payload[59] << 24;
            lastSample.velocityDown = (float)rawVelocityDown / 1e3;

            // Speed
            int32_t rawSpeed = frame.payload[60] | frame.payload[61] << 8 |
                               frame.payload[62] << 16 |
                               frame.payload[63] << 24;
            lastSample.speed = (float)rawSpeed / 1e3;

            // Track (heading of motion)
            int32_t rawTrack = frame.payload[64] | frame.payload[65] << 8 |
                               frame.payload[66] << 16 |
                               frame.payload[67] << 24;
            lastSample.track = (float)rawTrack / 1e5;

            // Number of satellite
            lastSample.satellites = (uint8_t)frame.payload[23];

            // Fix (every type of fix accepted)
            lastSample.fix = frame.payload[20] != 0;

            // Timestamp
            lastSample.gpsTimestamp =
                TimestampTimer::getInstance().getTimestamp();

            return true;
    }

    return false;
}

bool UbloxGPS::parseUBXACKFrame(const UBXUnpackedFrame& frame)
{
    switch (frame.id)  // Message ID
    {
        case 0x00:  // UBX-ACK-NAC
            LOG_DEBUG(logger,
                      "Received NAC for message (class: {:#02x}, id: {:#02x})",
                      frame.cls, frame.id);
            return true;
        case 0x01:  // UBX-ACK-ACK
            LOG_DEBUG(logger,
                      "Received ACK for message (class: {:#02x}, id: {:#02x})",
                      frame.cls, frame.id);
            return true;
    }
    return false;
}

bool UbloxGPS::writeUBXFrame(const UBXUnpackedFrame& frame)
{
    if (!frame.isValid())
    {
        LOG_ERR(logger, "UBX frame to write is invalid");
        return false;
    }

    SPITransaction spi{spiSlave};
    uint8_t packedFrame[UBX_MAX_FRAME_LENGTH];
    frame.writePacked(packedFrame);
    spi.write(packedFrame, frame.getLength());

    return true;
}

bool UbloxGPS::readUBXFrame(UBXUnpackedFrame& frame)
{
    SPITransaction spi{spiSlave};
    uint8_t packedFrame[UBX_MAX_FRAME_LENGTH];
    spi.read(packedFrame, UBX_MAX_FRAME_LENGTH);
    frame.readPacked(packedFrame);

    if (!frame.isValid())
    {
        LOG_ERR(logger, "UBX frame to read is invalid");
        return false;
    }

    return true;
}

}  // namespace Boardcore
