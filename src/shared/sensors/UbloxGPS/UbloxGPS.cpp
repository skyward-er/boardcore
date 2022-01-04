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
#include <drivers/serial.h>
#include <drivers/timer/TimestampTimer.h>
#include <fcntl.h>
#include <filesystem/file_access.h>

namespace Boardcore
{

using namespace miosix;

bool UbloxGPS::init()
{
    // Change the baud rate from the default value
    if (!setupCommunication())
    {
        return false;
    }

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

GPSData UbloxGPS::sampleImpl()
{
    Lock<FastMutex> l(sample_mutex);
    return last_sample;
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
    static constexpr uint16_t payload_length = 4;

    uint8_t payload[payload_length] = {
        0x00, 0x00,  // navBbrMask (Hot start)
        0x00,        // Hardware reset immediately
        0x00         // Reserved
    };

    UBXUnpackedFrame frame{0x06, 0x04,  // Message UBX-CFG-RST
                           payload, payload_length};

    return writeUBXFrame(frame);
}

bool UbloxGPS::disableNMEAMessages()
{
    static constexpr uint16_t payload_length = 34;

    uint8_t payload[payload_length] = {
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
                           payload, payload_length};

    return writeUBXFrame(frame);
}

bool UbloxGPS::setGNSSConfiguration()
{
    static constexpr uint16_t payload_length = 9;

    uint8_t payload[payload_length] = {
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x21, 0x00, 0x11, 0x20,  // CFG-NAVSPG-DYNMODEL key ID
        0x08                     // CFG-NAVSPG-DYNMODEL value
    };

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payload_length};

    return writeUBXFrame(frame);
}

bool UbloxGPS::enableUBXMessages()
{
    static constexpr uint16_t payload_length = 9;

    uint8_t payload[payload_length] = {
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x07, 0x00, 0x91, 0x20,  // CFG-MSGOUT-UBX_NAV_PVT_UART1 key ID
        0x01                     // CFG-MSGOUT-UBX_NAV_PVT_UART1 value
    };

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payload_length};

    return writeUBXFrame(frame);
}

bool UbloxGPS::setSampleRate()
{
    static constexpr uint16_t payload_length = 10;

    uint8_t payload[payload_length] = {
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x21, 0x30,  // CFG-RATE-MEAS key ID
        0xff, 0xff               // CFG-RATE-MEAS value (placeholder)
    };
    memcpy(&payload[8], &samplerate, 2);

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payload_length};

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
            // Lock the last_sample variable
            Lock<FastMutex> l(sample_mutex);

            // Latitude
            int32_t raw_latitude = frame.payload[28] | frame.payload[29] << 8 |
                                   frame.payload[30] << 16 |
                                   frame.payload[31] << 24;
            last_sample.latitude = (float)raw_latitude / 1e7;

            // Longitude
            int32_t raw_longitude = frame.payload[24] | frame.payload[25] << 8 |
                                    frame.payload[26] << 16 |
                                    frame.payload[27] << 24;
            last_sample.longitude = (float)raw_longitude / 1e7;

            // Height
            int32_t raw_height = frame.payload[32] | frame.payload[33] << 8 |
                                 frame.payload[34] << 16 |
                                 frame.payload[35] << 24;
            last_sample.height = (float)raw_height / 1e3;

            // Velocity north
            int32_t raw_velocity_north =
                frame.payload[48] | frame.payload[49] << 8 |
                frame.payload[50] << 16 | frame.payload[51] << 24;
            last_sample.velocity_north = (float)raw_velocity_north / 1e3;

            // Velocity east
            int32_t raw_velocity_east =
                frame.payload[52] | frame.payload[53] << 8 |
                frame.payload[54] << 16 | frame.payload[55] << 24;
            last_sample.velocity_east = (float)raw_velocity_east / 1e3;

            // Velocity down
            int32_t raw_velocity_down =
                frame.payload[56] | frame.payload[57] << 8 |
                frame.payload[58] << 16 | frame.payload[59] << 24;
            last_sample.velocity_down = (float)raw_velocity_down / 1e3;

            // Speed
            int32_t raw_speed = frame.payload[60] | frame.payload[61] << 8 |
                                frame.payload[62] << 16 |
                                frame.payload[63] << 24;
            last_sample.speed = (float)raw_speed / 1e3;

            // Track (heading of motion)
            int32_t raw_track = frame.payload[64] | frame.payload[65] << 8 |
                                frame.payload[66] << 16 |
                                frame.payload[67] << 24;
            last_sample.track = (float)raw_track / 1e5;

            // Number of satellite
            last_sample.num_satellites = (uint8_t)frame.payload[23];

            // Fix (every type of fix accepted)
            last_sample.fix = frame.payload[20] != 0;

            // Timestamp
            last_sample.gps_timestamp =
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

    uint8_t packed_frame[UBX_MAX_FRAME_LENGTH];
    frame.writePacked(packed_frame);

    writeRaw(packed_frame, frame.getFrameLength());

    return true;
}

bool UbloxGPS::readUBXFrame(UBXUnpackedFrame& frame)
{
    bool synchronized = false;
    while (!synchronized)
    {
        synchronized = true;
        for (uint16_t i = 0; synchronized && i < 2; ++i)
        {
            if (!readRaw(&frame.preamble[i], 1))
                return false;

            if (frame.preamble[i] != UBX_VALID_PREAMBLE[i])
                synchronized = false;
        }
    }

    if (!readRaw(&frame.cls, 1) || !readRaw(&frame.id, 1) ||
        !readRaw((uint8_t*)&frame.payload_length, 2) ||
        !readRaw(frame.payload, frame.payload_length) ||
        !readRaw(frame.checksum, 2))
        return false;

    if (!frame.isValid())
    {
        LOG_ERR(logger, "UBX frame to read is invalid");
        return false;
    }

    return true;
}

UbloxGPSSPI::UbloxGPSSPI(SPIBusInterface& spi_bus, GpioPin spi_cs,
                         SPIBusConfig spi_config, uint8_t samplerate)
    : UbloxGPS(samplerate), spi_slave(spi_bus, spi_cs, spi_config)
{
}

SPIBusConfig UbloxGPSSPI::getDefaultSPIConfig()
{
    SPIBusConfig spi_config{};
    spi_config.clockDivider = SPI::ClockDivider::DIV_32;
    spi_config.mode         = SPI::Mode::MODE_1;
    return spi_config;
}

bool UbloxGPSSPI::writeRaw(uint8_t* data, size_t size)
{
    SPITransaction spi{spi_slave};
    spi.write(data, size);
    return true;
}

bool UbloxGPSSPI::readRaw(uint8_t* data, size_t size)
{
    SPITransaction spi{spi_slave};
    spi.read(data, size);
    return true;
}

UbloxGPSSerial::UbloxGPSSerial(int serial_port_number,
                               const char* serial_port_name,
                               int serial_baudrate, int serial_default_baudrate,
                               uint8_t samplerate)
    : UbloxGPS(samplerate), serial_port_number(serial_port_number),
      serial_port_name(serial_port_name), serial_baudrate(serial_baudrate),
      serial_default_baudrate(serial_default_baudrate)
{
    strcpy(serial_file_path, "/dev/");
    strcat(serial_file_path, serial_port_name);
}

bool UbloxGPSSerial::setupCommunication()
{
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();

    // Close the serial file if already opened
    devFs->remove(serial_port_name);

    // Change the baudrate only if it is different than the default
    if (serial_baudrate != serial_default_baudrate)
    {
        // Open the serial port device with the default baudrate
        if (!devFs->addDevice(
                serial_port_name,
                intrusive_ref_ptr<Device>(new STM32Serial(
                    serial_port_number, serial_default_baudrate))))
        {
            LOG_ERR(logger,
                    "[gps] Faild to open serial port {} with baudrate {} as "
                    "file {}",
                    serial_port_number, serial_default_baudrate,
                    serial_port_name);
            return false;
        }

        // Open the serial file
        if ((serial_file = open(serial_file_path, O_RDWR)) < 0)
        {
            LOG_ERR(logger, "Failed to open serial file {}", serial_file_path);
            return false;
        }

        // Change baudrate
        if (!setBaudrate())
        {
            return false;
        }

        // Close the serial file
        if (close(serial_file) < 0)
        {
            LOG_ERR(logger, "Failed to close serial file {}", serial_file_path);
            return false;
        }

        // Close the serial port
        if (!devFs->remove(serial_port_name))
        {
            LOG_ERR(logger, "Failed to close serial port {} as file {}",
                    serial_port_number, serial_port_name);
            return false;
        }
    }

    // Reopen the serial port with the configured baudrate
    if (!devFs->addDevice(serial_port_name,
                          intrusive_ref_ptr<Device>(new STM32Serial(
                              serial_port_number, serial_baudrate))))
    {
        LOG_ERR(logger,
                "Faild to open serial port {} with baudrate {} as file {}\n",
                serial_port_number, serial_default_baudrate, serial_port_name);
        return false;
    }

    // Reopen the serial file
    if ((serial_file = open(serial_file_path, O_RDWR)) < 0)
    {
        LOG_ERR(logger, "Failed to open serial file {}", serial_file_path);
        return false;
    }

    return true;
}

bool UbloxGPSSerial::setBaudrate()
{
    static constexpr uint16_t payload_length = 12;

    uint8_t payload[payload_length] = {
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x52, 0x40,  // Configuration item key ID
        0xff, 0xff, 0xff, 0xff   // Value (placeholder)
    };
    memcpy(&payload[8], &serial_baudrate, 4);

    UBXUnpackedFrame frame{0x06, 0x8a,  // Message UBX-CFG-VALSET
                           payload, payload_length};

    return writeUBXFrame(frame);
}

bool UbloxGPSSerial::writeRaw(uint8_t* data, size_t size)
{
    return write(serial_file, data, size) >= 0;
}

bool UbloxGPSSerial::readRaw(uint8_t* data, size_t size)
{
    return read(serial_file, data, size) >= 0;
}

}  // namespace Boardcore
