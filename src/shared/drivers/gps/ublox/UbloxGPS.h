/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Davide Bonomini
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

#ifndef GPS_H
#define GPS_H

#include <fcntl.h>

#include "ActiveObject.h"
#include "Common.h"
#include "UbloxGPSData.h"
#include "sensors/Sensor.h"

using miosix::FastMutex;
using miosix::Thread;

class UbloxGPS : public Sensor<UbloxGPSData>, public ActiveObject
{
public:
    /*
     * Which time reference to use
     */
    enum class TimeRef
    {
        UTC     = 0,
        GPS     = 1,
        GLONASS = 2,
        BEIDOU  = 3,
        GALILEO = 4
    };

    UbloxGPS(int baudrate = 460800, int sampleRate = 10, int serialPortNum = 2,
        const char *portName = "gps");

    /*
     * @brief Read from the GPS serial port and try to parse the NMEA messages
     * @return 1 true if at least one NMEA message was correctly parsed, 0 if no
     * valid NMEA messages were received
     */
    bool selfTest() override;

    /*
     * @brief Sets up the serial port and sends the UBX configuration messages
     * to the GPS module
     * @return True if the serial port was opened correctly and false if some
     * error occured
     */
    bool init() override;

    UbloxGPSData sampleImpl() override;

    /*
     * Packs and sends a UBX configuration message to the GPS module to set up
     * SBAS. By default it sets up EGNOS.
     */
    void sendSBASMessage(int mode = 1, int usage = 3, int maxChannelNum = 3,
                         int PRNs[3] = NULL);

    /*
     * Packs and sends a UBX configuration message to the GPS module to set the
     * rate. By default it samples the gps every 100 ms and uses GPS time
     */
    void sendRateMessage(int inbetweenReadings = 100, int navRate = 1,
                         TimeRef timeRef = TimeRef::GPS);

private:
    /*
     * Active object run
     */
    void run() override;

    /*
     * Sets up the serial port and sends UBX configuration messages to the GPS
     * module
     */
    bool serialComSetup();

    /*
     * Gets called when selfTest() gets called and selfTestFlag is set to true
     */
    bool selfTestInThread();

    /*
     * Writes the UBX checksum in the last 2 bytes of msg
     */
    void ubxChecksum(uint8_t *msg, int len);

    /*
     * Packs SBAS message:
     *
     * Length: 16 bytes
     * Supported on UBX protocol versions  15, 15.01, 16, 17, 18,
     * 19, 19.1, 19.2, 20, 20.01, 20.1, 20.2, 20.3, 22, 23 and 23.01 bytes 0-5
     * as UBX standard (header x2, class x1, id x1, length x2) byte 6 mode,
     * default 1 (deprecated -> has no effect), mode 2 (test mode) ignores SBAS
     * data byte 7 usage, default 3 (correction and ranging) byte 8 max
     * channels, default 3 (valid range: 0-3) byte 9-13 PRNs, default for EGNOS:
     * 136, 123, 126 (as of March 2020) btye 14-15 checksum
     */
    void packSBASMessge(uint8_t *msg, int mode = 1, int usage = 3,
                        int maxChannelNum = 3, int PRNs[3] = NULL);

    /*
     * Packs Rate message:
     *
     * Length: 14 bytes
     * bytes 0-5 as UBX standard (header x2, class x1, id x1, length x2)
     * bytes 6-7 measurement rate, time between readings, default 100
     * bytes 8-9 navRate, measurements taken before creating a solution, default
     * 1 bytes timeRef, Alignment to reference time, default GPS
     */
    void packRateMessage(uint8_t *msg, int inbetweenReadings = 100,
                         int navRate = 1, TimeRef timeRef = TimeRef::GPS);

    struct UbloxGPSData data;
    mutable FastMutex mutex;
    int fd, baudrate, serialPortNum, betweenReadings;
    volatile bool selfTestResult, selfTestFlag = false;
    const char *serialPortName;

    static const int MAXPORTNAMELEN       = 5;
    static const int DEFAULT_GPS_BAUDRATE = 38400;
    static const int MAX_SAMPLERATE       = 25;
    static const int NMEA_MAX_LENGTH      = 82;
    static const int SELFTEST_TIMEOUT     = 2000;
    static const int SET_BAUDRATE_MSG_LEN = 28;
    static const int SET_RATE_MSG_LEN     = 14;
    static const int SET_SBAS_MSG_LEN     = 16;

    static constexpr uint8_t UbxHeader[2] = {0xb5, 0x62};
    static const uint8_t UbxCfgClass      = 0x06;
    static const uint8_t UbxCfgId_SBAS    = 0x16;
    static const int UbxCfgLen_SBAS       = 8;
    static const uint8_t UbxCfgId_Rate    = 0x08;
    static const int UbxCfgLen_Rate       = 6;

    static constexpr uint8_t SETGNSS[44] = {
        0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff, 0x08, 0x03, 0x00, 0x00,
        0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xfa, 0x00, 0xfa, 0x00,
        0x64, 0x00, 0x2c, 0x01, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x54, 0x2c};  // set GNSS options
                                                          // (GPS channels,
                                                          // GLONASS channels,
                                                          // EGNOS chanels and
                                                          // navigation mode to
                                                          // airborne <4g)

    static constexpr uint8_t DISABLEGSA[16] = {
        0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};

    static constexpr uint8_t DISABLEGLL[16] = {
        0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2b};

    static constexpr uint8_t DISABLEGSV[16] = {
        0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x03,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};

    static constexpr uint8_t DISABLEVTG[16] = {
        0xb5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xf0, 0x05,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
};

#endif /* GPS_H */
