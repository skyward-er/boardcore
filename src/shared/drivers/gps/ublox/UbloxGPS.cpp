/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio
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
#include <fcntl.h>
#include <filesystem/file_access.h>

using namespace miosix;

namespace Boardcore
{

UbloxGPS::UbloxGPS(int baudrate_, uint8_t sampleRate_, int serialPortNum_,
                   const char* serialPortName_, int defaultBaudrate_)
    : baudrate(baudrate_), sampleRate(sampleRate_),
      serialPortNumber(serialPortNum_), serialPortName(serialPortName_),
      defaultBaudrate(defaultBaudrate_)
{
    // Prepare the gps file path with the specified name
    strcpy(gpsFilePath, "/dev/");
    strcat(gpsFilePath, serialPortName);
}

bool UbloxGPS::init()
{
    // Change the baud rate from the default value
    if (!serialCommuinicationSetup())
    {
        return false;
    }

    Thread::sleep(10);

    // Reset configuration to default
    // TODO: maybe move this on serial communication setup
    if (!resetConfiguration())
    {
        return false;
    }

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

    // Set rate
    if (!setRate())
    {
        return false;
    }

    return true;
}

bool UbloxGPS::selfTest() { return true; }

UbloxGPSData UbloxGPS::sampleImpl()
{
    Lock<FastMutex> l(mutex);
    return threadSample;
}

void UbloxGPS::run()
{
    /**
     * UBX message structure:
     * - 2B: Preamble
     * - 1B: Message class
     * - 1B: Message id
     * - 2B: Payload length
     * - lB: Payload
     * - 2B: Checksum
     */
    uint8_t message[6 + UBX_MAX_PAYLOAD_LENGTH + 2];
    uint16_t payloadLength;

    while (!shouldStop())
    {
        StackLogger::getInstance()->updateStack(THID_GPS);

        // Try to read the message
        if (!readUBXMessage(message, payloadLength))
        {
            LOG_DEBUG(logger, "Unable to read a UBX message");
            continue;
        }

        // Parse the message
        if (!parseUBXMessage(message))
        {
            LOG_DEBUG(logger,
                      "UBX message not recognized (class:0x{02x}, id: 0x{02x})",
                      message[2], message[3]);
        }
    }
}

void UbloxGPS::ubxChecksum(uint8_t* msg, int len)
{
    uint8_t ck_a = 0, ck_b = 0;

    // The length must be valid, at least 8 bytes (preamble, msg, length,
    // checksum)
    if (len <= 8)
    {
        return;
    }

    // Checksum calculation from byte 2 to end of payload
    for (int i = 2; i < len - 2; i++)
    {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }

    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;
}

bool UbloxGPS::writeUBXMessage(uint8_t* message, int length)
{
    // Compute the checksum
    ubxChecksum(message, length);

    // Write configuration
    if (write(gpsFile, message, length) < 0)
    {
        LOG_ERR(logger,
                "Failed to write ubx message (class:0x{02x}, id: 0x{02x})",
                message[2], message[3]);
        return false;
    }

    return true;
}

bool UbloxGPS::serialCommuinicationSetup()
{
    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();

    // Change the baudrate only if it is different than the default
    if (baudrate != defaultBaudrate)
    {
        // Close the gps file if already opened
        devFs->remove(serialPortName);

        // Open the serial port device with the default boudrate
        if (!devFs->addDevice(serialPortName,
                              intrusive_ref_ptr<Device>(new STM32Serial(
                                  serialPortNumber, defaultBaudrate))))
        {
            LOG_ERR(logger,
                    "[gps] Faild to open serial port {0} with baudrate {1} as "
                    "file {2}",
                    serialPortNumber, defaultBaudrate, serialPortName);
            return false;
        }

        // Open the gps file
        if ((gpsFile = open(gpsFilePath, O_RDWR)) < 0)
        {
            LOG_ERR(logger, "Failed to open gps file {}", gpsFilePath);
            return false;
        }

        // Change boudrate
        if (!setBaudrate())
        {
            return false;
        };

        // Close the gps file
        if (close(gpsFile) < 0)
        {
            LOG_ERR(logger, "Failed to close gps file {}", gpsFilePath);
            return false;
        }

        // Close the serial port
        if (!devFs->remove(serialPortName))
        {
            LOG_ERR(logger, "Failed to close serial port {} as file {}",
                    serialPortNumber, serialPortName);
            return false;
        }
    }

    // Reopen the serial port with the configured boudrate
    if (!devFs->addDevice(serialPortName,
                          intrusive_ref_ptr<Device>(
                              new STM32Serial(serialPortNumber, baudrate))))
    {
        LOG_ERR(logger,
                "Faild to open serial port {} with baudrate {} as file {}\n",
                serialPortNumber, defaultBaudrate, serialPortName);
        return false;
    }

    // Reopen the gps file
    if ((gpsFile = open(gpsFilePath, O_RDWR)) < 0)
    {
        LOG_ERR(logger, "Failed to open gps file {}", gpsFilePath);
        return false;
    }

    return true;
}

bool UbloxGPS::resetConfiguration()
{
    uint8_t ubx_cfg_prt[RESET_CONFIG_MSG_LEN] = {
        0Xb5, 0x62,  // Preamble
        0x06, 0x04,  // Message UBX-CFG-RST
        0x04, 0x00,  // Length
        0x00, 0x00,  // navBbrMask (Hot start)
        0x00,        // Hardware reset immediately
        0x00,        // Reserved
        0xff, 0xff   // Checksum
    };

    return writeUBXMessage(ubx_cfg_prt, RESET_CONFIG_MSG_LEN);
}

bool UbloxGPS::setBaudrate()
{
    uint8_t ubx_cfg_prt[SET_BAUDRATE_MSG_LEN] = {
        0Xb5, 0x62,              // Preamble
        0x06, 0x8a,              // Message UBX-CFG-VALSET
        0x0c, 0x00,              // Length
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x52, 0x40,  // Configuration item key ID
        0xff, 0xff, 0xff, 0xff,  // Value
        0xff, 0xff               // Checksum
    };

    // Prepare boud rate
    ubx_cfg_prt[14] = baudrate;
    ubx_cfg_prt[15] = baudrate >> 8;
    ubx_cfg_prt[16] = baudrate >> 16;
    ubx_cfg_prt[17] = baudrate >> 24;

    return writeUBXMessage(ubx_cfg_prt, SET_BAUDRATE_MSG_LEN);
}

bool UbloxGPS::disableNMEAMessages()
{
    uint8_t ubx_cfg_valset[DISABLE_NMEA_MESSAGES_MSG_LEN] = {
        0Xb5, 0x62,              // Preamble
        0x06, 0x8a,              // Message UBX-CFG-VALSET
        0x22, 0x00,              // Length
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
        0x00,                    // CFG-MSGOUT-NMEA_ID_VTG_UART1 value
        0xff, 0xff               // Checksum
    };

    return writeUBXMessage(ubx_cfg_valset, DISABLE_NMEA_MESSAGES_MSG_LEN);
}

bool UbloxGPS::setGNSSConfiguration()
{
    uint8_t ubx_cfg_valset[SET_GNSS_CONF_LEN] = {
        0Xb5, 0x62,              // Preamble
        0x06, 0x8a,              // Message UBX-CFG-VALSET
        0x09, 0x00,              // Length
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x21, 0x00, 0x11, 0x20,  // CFG-NAVSPG-DYNMODEL key ID
        0x08,                    // CFG-NAVSPG-DYNMODEL value
        0xff, 0xff               // Checksum
    };

    return writeUBXMessage(ubx_cfg_valset, SET_GNSS_CONF_LEN);
}

bool UbloxGPS::enableUBXMessages()
{
    uint8_t ubx_cfg_valset[ENABLE_UBX_MESSAGES_MSG_LEN] = {
        0Xb5, 0x62,              // Preamble
        0x06, 0x8a,              // Message UBX-CFG-VALSET
        0x09, 0x00,              // Length
        0x00,                    // Version
        0xff,                    // All layers
        0x00, 0x00,              // Reserved
        0x07, 0x00, 0x91, 0x20,  // CFG-MSGOUT-UBX_NAV_PVT_UART1 key ID
        0x01,                    // CFG-MSGOUT-UBX_NAV_PVT_UART1 value
        0xff, 0xff               // Checksum
    };

    return writeUBXMessage(ubx_cfg_valset, ENABLE_UBX_MESSAGES_MSG_LEN);
}

bool UbloxGPS::setRate()
{
    uint16_t rate = 1000 / sampleRate;
    LOG_DEBUG(logger, "Rate: {}", rate);

    uint8_t ubx_cfg_valset[SET_RATE_MSG_LEN] = {
        0Xb5, 0x62,              // Preamble
        0x06, 0x8a,              // Message UBX-CFG-VALSET
        0x0a, 0x00,              // Length
        0x00,                    // Version
        0x07,                    // All layers
        0x00, 0x00,              // Reserved
        0x01, 0x00, 0x21, 0x30,  // CFG-RATE-MEAS key ID
        0xff, 0xff,              // CFG-RATE-MEAS value
        0xff, 0xff               // Checksum
    };

    // Prepare rate
    ubx_cfg_valset[14] = rate;
    ubx_cfg_valset[15] = rate >> 8;

    return writeUBXMessage(ubx_cfg_valset, SET_RATE_MSG_LEN);
}

bool UbloxGPS::readUBXMessage(uint8_t* message, uint16_t& payloadLength)
{
    // Read preamble
    do
    {
        // Read util the first byte of the preamble
        do
        {
            if (read(gpsFile, &message[0], 1) <= 0)  // No more data available
            {
                return false;
            }
        } while (message[0] != PREAMBLE[0]);

        // Read the next byte
        if (read(gpsFile, &message[1], 1) <= 0)  // No more data available
        {
            return false;
        }
    } while (message[1] != PREAMBLE[1]);  // Continue

    // Read message class and ID
    if (read(gpsFile, &message[2], 1) <= 0)
    {
        return false;
    }
    if (read(gpsFile, &message[3], 1) <= 0)
    {
        return false;
    }

    // Read length
    if (read(gpsFile, &message[4], 2) <= 0)
    {
        return false;
    }
    payloadLength = message[4] | (message[5] << 8);
    if (payloadLength > UBX_MAX_PAYLOAD_LENGTH)
    {
        return false;
    }

    // Read paylaod and checksum
    for (auto i = 0; i < payloadLength + 2; i++)
    {
        if (read(gpsFile, &message[6 + i], 1) <= 0)
        {
            return false;
        }
    }

    // Verify the checksum
    uint8_t msgChecksum1 = message[6 + payloadLength];
    uint8_t msgChecksum2 = message[6 + payloadLength + 1];
    ubxChecksum(message, 6 + payloadLength + 2);
    if (msgChecksum1 != message[6 + payloadLength] ||
        msgChecksum2 != message[6 + payloadLength + 1])
    {
        LOG_ERR(logger, "Message checksum verification failed");
        return false;
    }

    return true;
}

bool UbloxGPS::parseUBXMessage(uint8_t* message)
{
    switch (message[2])  // Message class
    {
        case 0x01:  // UBX-NAV
            return parseUBXNAVMessage(message);
        case 0x05:  // UBX-ACK
            return parseUBXACKMessage(message);
    }
    return false;
}

bool UbloxGPS::parseUBXNAVMessage(uint8_t* message)
{
    switch (message[3])  // Message id
    {
        case 0x07:  // UBX-NAV-PVT
            // Lock the threadSample variable
            Lock<FastMutex> l(mutex);

            // Latitude
            int32_t rawLatitude = message[6 + 28] | message[6 + 29] << 8 |
                                  message[6 + 30] << 16 | message[6 + 31] << 24;
            threadSample.latitude = (float)rawLatitude / 1e7;

            // Longitude
            int32_t rawLongitude = message[6 + 24] | message[6 + 25] << 8 |
                                   message[6 + 26] << 16 |
                                   message[6 + 27] << 24;
            threadSample.longitude = (float)rawLongitude / 1e7;

            // Height
            int32_t rawHeight = message[6 + 32] | message[6 + 33] << 8 |
                                message[6 + 34] << 16 | message[6 + 35] << 24;
            threadSample.height = (float)rawHeight / 1e3;

            // Velocity north
            int32_t rawVelocityNorth = message[6 + 48] | message[6 + 49] << 8 |
                                       message[6 + 50] << 16 |
                                       message[6 + 51] << 24;
            threadSample.velocity_north = (float)rawVelocityNorth / 1e3;

            // Velocity east
            int32_t rawVelocityEast = message[6 + 52] | message[6 + 53] << 8 |
                                      message[6 + 54] << 16 |
                                      message[6 + 55] << 24;
            threadSample.velocity_east = (float)rawVelocityEast / 1e3;

            // Velocity down
            int32_t rawVelocityDown = message[6 + 56] | message[6 + 57] << 8 |
                                      message[6 + 58] << 16 |
                                      message[6 + 59] << 24;
            threadSample.velocity_down = (float)rawVelocityDown / 1e3;

            // Speed
            int32_t rawSpeed = message[6 + 60] | message[6 + 61] << 8 |
                               message[6 + 62] << 16 | message[6 + 63] << 24;
            threadSample.speed = (float)rawSpeed / 1e3;

            // Track (heading of motion)
            int32_t rawTrack = message[6 + 64] | message[6 + 65] << 8 |
                               message[6 + 66] << 16 | message[6 + 67] << 24;
            threadSample.track = (float)rawTrack / 1e5;

            // Number of satellite
            threadSample.num_satellites = (uint8_t)message[6 + 23];

            // Fix (every type of fix accepted)
            threadSample.fix = message[6 + 20] != 0;

            // Timestamp
            threadSample.gps_timestamp = TimestampTimer::getTimestamp();

            return true;
    }

    return false;
}

bool UbloxGPS::parseUBXACKMessage(uint8_t* message)
{
    switch (message[3])  // Message id
    {
        case 0x00:  // UBX-ACK-NAC
            LOG_DEBUG(logger,
                      "Received NAC for message (class:0x{02x}, id: 0x{02x})",
                      message[6], message[7]);
            return true;
        case 0x01:  // UBX-ACK-ACK
            LOG_DEBUG(logger,
                      "Received ACK for message (class:0x{02x}, id: 0x{02x})",
                      message[6], message[7]);
            return true;
    }
    return false;
}

}  // namespace Boardcore
