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

#pragma once

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include "UbloxGPSData.h"

namespace Boardcore
{

/**
 * @brief Driver for Ublox GPSs
 *
 * This driver handles communication and setup with Ublox GPSs. It uses the
 * binary UBX protocol to retrieve and parse navigation data quicker than using
 * the string based NMEA.
 *
 * At initialization it configures the device with the specified baudrate, reset
 * the configuration and sets up UBX messages and GNSS parameters.
 *
 * Communication with the device is performed through a file, the driver opens
 * the serial port under the filepath /dev/<serialPortName>.
 * There is no need for the file to be setted up beforhand.
 *
 * This driver was written for a NEO-M9N gps with the latest version of UBX.
 */
class UbloxGPS : public Sensor<UbloxGPSData>, public ActiveObject
{
public:
    /**
     * @brief Construct a new UbloxGPS object
     *
     * @param boudrate_ Baudrate to communicate with the device (max: 921600,
     * min: 4800 for NEO-M9N)
     * @param sampleRate_ GPS sample rate (max: 25 for NEO-M9N)
     * @param serialPortName_ Name of the file for the gps device
     * @param defaultBaudrate_ Startup baudrate (38400 for NEO-M9N)
     */
    UbloxGPS(int boudrate_ = 921600, uint8_t sampleRate_ = 10,
             int serialPortNumber_ = 2, const char *serialPortName_ = "gps",
             int defaultBaudrate_ = 38400);

    /**
     * @brief Sets up the serial port baudrate, disables the NMEA messages,
     * configures GNSS options and enables UBX-PVT message
     *
     * @return True if the operation succeeded
     */
    bool init() override;

    /**
     * @brief Read a single message form the GPS, waits 2 sample cycle
     *
     * @return True if a message was sampled
     */
    bool selfTest() override;

private:
    UbloxGPSData sampleImpl() override;

    void run() override;

    void ubxChecksum(uint8_t *msg, int len);

    /**
     * @brief Compute the checksum and write the message to the device
     */
    bool writeUBXMessage(uint8_t *message, int length);

    /**
     * @brief Sets up the serial port with the correct baudrate
     *
     * Opens the serial port with the defaul baudrate and changes it to
     * the value specified in the constructor, then it reopens the serial port.
     * If the device is already using the correct baudrate this won't have
     * effect. However if the gps is using a different baudrate the diver won't
     * be able to communicate.
     */
    bool serialCommuinicationSetup();

    bool resetConfiguration();

    bool setBaudrate();

    bool disableNMEAMessages();

    bool setGNSSConfiguration();

    bool enableUBXMessages();

    bool setRate();

    bool readUBXMessage(uint8_t *message, uint16_t &payloadLength);

    bool parseUBXMessage(uint8_t *message);

    bool parseUBXNAVMessage(uint8_t *message);

    bool parseUBXACKMessage(uint8_t *message);

    const int baudrate;        // [baud]
    const uint8_t sampleRate;  // [Hz]
    const int serialPortNumber;
    const char *serialPortName;
    const int defaultBaudrate;  // [baud]

    char gpsFilePath[16];  ///< Allows for a filename of up to 10 characters
    int gpsFile = -1;

    mutable miosix::FastMutex mutex;
    UbloxGPSData threadSample{};

    static constexpr int SET_BAUDRATE_MSG_LEN          = 20;
    static constexpr int RESET_CONFIG_MSG_LEN          = 12;
    static constexpr int DISABLE_NMEA_MESSAGES_MSG_LEN = 42;
    static constexpr int SET_GNSS_CONF_LEN             = 17;
    static constexpr int ENABLE_UBX_MESSAGES_MSG_LEN   = 17;
    static constexpr int SET_RATE_MSG_LEN              = 18;

    static constexpr uint8_t PREAMBLE[] = {0xb5, 0x62};

    static constexpr int UBX_MAX_PAYLOAD_LENGTH = 92;

    PrintLogger logger = Logging::getLogger("ubloxgps");
};

}  // namespace Boardcore
