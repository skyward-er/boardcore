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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <miosix.h>

#include "ActiveObject.h"
#include "UbloxGPSData.h"
#include "sensors/Sensor.h"

class UbloxGPS : public Sensor<UbloxGPSData>, public ActiveObject
{
public:
    UbloxGPS(int boudrate_ = 921600, uint8_t sampleRate_ = 10,
             int serialPortNumber_ = 2, const char *serialPortName_ = "gps",
             int defaultBaudrate_ = 38400);

    bool init() override;

    bool selfTest() override;

private:
    UbloxGPSData sampleImpl() override;

    void run() override;

    bool selfTestInThread();

    void ubxChecksum(uint8_t *msg, int len);

    /**
     * Also compute the checksum
     */
    bool writeUBXMessage(uint8_t *message, int length);

    bool serialCommuinicationSetup();

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
    int gpsFile;

    mutable miosix::FastMutex mutex;
    UbloxGPSData threadSample{};

    static constexpr int SET_BAUDRATE_MSG_LEN          = 20;
    static constexpr int DISABLE_NMEA_MESSAGES_MSG_LEN = 42;
    static constexpr int SET_GNSS_CONF_LEN             = 17;
    static constexpr int ENABLE_UBX_MESSAGES_MSG_LEN   = 17;
    static constexpr int SET_RATE_MSG_LEN              = 18;

    static constexpr uint8_t PREAMBLE[] = {0xb5, 0x62};

    static constexpr int UBX_MAX_PAYLOAD_LENGTH = 92;
};
