/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Davide Bonomini
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

#include <drivers/gps/Gps.h>
#include <drivers/gps/nmea/nmea.h>

#include <cmath>

#include "drivers/serial.h"
#include "filesystem/file_access.h"

using namespace miosix;

constexpr uint8_t Gps::DISABLEGSA[];
constexpr uint8_t Gps::DISABLEGSV[];
constexpr uint8_t Gps::DISABLEVTG[];
constexpr uint8_t Gps::DISABLEGLL[];
constexpr uint8_t Gps::SETGNSS[];

Gps::Gps(int baudrate, int sampleRate, int serialPortNum,
         const char* serialPortName)
{
    Gps::baudrate       = baudrate;
    Gps::serialPortNum  = serialPortNum;
    Gps::serialPortName = serialPortName;

    sampleRate           = std::min(sampleRate, Gps::MAX_SAMPLERATE);
    Gps::betweenReadings = 1000 / sampleRate;

    data.gps_timestamp = 0;
    data.velocity_down = 0;
    data.fix           = false;
}

bool Gps::init() { return serialComSetup(); }

void Gps::run()
{
    int deg;
    enum minmea_sentence_id sentence_id;
    struct minmea_sentence_gga frame_gga;
    struct minmea_sentence_rmc frame_rmc;
    char msg[NMEA_MAX_LENGTH + 1];
    float trackDegrees;

    while (!shouldStop())
    {
        if (selfTestFlag)
        {
            selfTestResult = selfTestInThread();
            selfTestFlag   = false;
        }

        int i = 0;
        read(fd, msg, 1);
        while (msg[i] != '\n')
        {
            i++;
            read(fd, &msg[i], 1);
        }
        msg[++i] = '\0';

        sentence_id = minmea_sentence_id(msg, 0);

        /*
        nmea messages active on gps module:
        MINMEA_SENTENCE_RMC,
        MINMEA_SENTENCE_GGA
        */

        Lock<FastMutex> l(mutex);  // update struct
        switch (sentence_id)
        {
            case MINMEA_SENTENCE_RMC:
                if (minmea_parse_rmc(&frame_rmc, msg))
                {
                    if (frame_rmc.latitude.value == 0 &&
                        frame_rmc.longitude.value == 0)
                        data.fix = false;
                    else
                    {
                        {
                            data.fix           = true;
                            data.gps_timestamp = getTick();
                            deg                = frame_rmc.latitude.value /
                                  frame_rmc.latitude.scale / 100;
                            data.latitude =
                                deg + ((float)frame_rmc.latitude.value /
                                           frame_rmc.latitude.scale / 100 -
                                       deg) /
                                          60 *
                                          100;  // conversion from degrees and
                                                // minutes to decimal degrees
                            deg = frame_rmc.longitude.value /
                                  frame_rmc.longitude.scale / 100;
                            data.longitude =
                                deg + ((float)frame_rmc.longitude.value /
                                           frame_rmc.longitude.scale / 100 -
                                       deg) /
                                          60 *
                                          100;  // conversion from degrees and
                                                // minutes to decimal degrees
                            data.speed = ((float)frame_rmc.speed.value /
                                          frame_rmc.speed.scale) *
                                         KNOTS_TO_MPS;
                            if (frame_rmc.course.scale == 0 &&
                                frame_rmc.course.value == 0)
                            {
                                data.track          = 0;
                                data.velocity_north = 0;
                                data.velocity_east  = 0;
                            }
                            else
                            {
                                data.track = (float)frame_rmc.course.value /
                                             frame_rmc.course.scale;

                                trackDegrees = data.track * DEGREES_TO_RADIANS;
                                data.velocity_north =
                                    cos(trackDegrees) * data.speed;
                                data.velocity_east =
                                    sin(trackDegrees) * data.speed;
                            }
                        }
                    }
                }
                break;

            case MINMEA_SENTENCE_GGA:
                if (minmea_parse_gga(&frame_gga, msg))
                {
                    if (data.fix)
                    {
                        {
                            data.height = (float)frame_gga.altitude.value /
                                          frame_gga.altitude.scale;

                            data.num_satellites = frame_gga.satellites_tracked;
                        }
                    }
                }
                break;

            default:
                TRACE("Unrecognized NMEA message: %s", msg);
                break;
        }
    }
}

bool Gps::serialComSetup()
{
    uint8_t setBaudRateMsg[SET_BAUDRATE_MSG_LEN] = {
        0Xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00,
        0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff};

    intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();
    if (!devFs->remove(serialPortName))
        return false;
    if (!devFs->addDevice(serialPortName,
                          intrusive_ref_ptr<Device>(new STM32Serial(
                              serialPortNum, DEFAULT_GPS_BAUDRATE))))
        return false;

    char serialPortPath[10];
    strcpy(serialPortPath, "/dev/");
    strcat(serialPortPath, serialPortName);
    /* open serial port named gps*/
    fd = open(serialPortPath, O_RDWR);
    if (fd < 0)
        printf("Cannot open %s", serialPortPath);

    /* send configuration messages */
    write(fd, DISABLEGSA, 16);

    write(fd, DISABLEGLL, 16);

    write(fd, DISABLEGSV, 16);

    write(fd, DISABLEVTG, 16);

    write(fd, SETGNSS, 44);

    /*
    construct and send rate messge
    bytes 6 and 7 for rate
    last 2 bytes for checksum
    */
    uint8_t msg[SET_RATE_MSG_LEN];
    packRateMessage(msg, betweenReadings);
    write(fd, msg, SET_RATE_MSG_LEN);

    /*
    construct and send baudrate message
    bytes 14 to 17 for baudrate
    last 2 bytes for checksum
    */
    setBaudRateMsg[14] = baudrate & 0x000000ff;
    setBaudRateMsg[15] = (baudrate & 0x0000ff00) >> 8;
    setBaudRateMsg[16] = (baudrate & 0x00ff0000) >> 16;
    setBaudRateMsg[17] = (baudrate & 0xff000000) >> 24;
    ubxChecksum(setBaudRateMsg, SET_BAUDRATE_MSG_LEN);
    write(fd, setBaudRateMsg, SET_BAUDRATE_MSG_LEN);

    /* close and reopen serial port with new baudrate */
    close(fd);
    if (!devFs->remove(serialPortName))
        return false;
    if (!devFs->addDevice(serialPortName,
                          intrusive_ref_ptr<Device>(
                              new STM32Serial(serialPortNum, baudrate))))
        return false;

    fd = open(serialPortPath, O_RDWR);
    if (fd < 0)
        printf("Cannot open %s", serialPortPath);

    return true;
}

bool Gps::selfTest()
{
    int start;
    selfTestFlag = true;
    start        = getTick();
    Thread::sleep(20);
    while (selfTestFlag)
    {
        if (getTick() - start > SELFTEST_TIMEOUT)
            return false;
        Thread::sleep(20);
    }

    return selfTestResult;
}

bool Gps::selfTestInThread()
{
    enum minmea_sentence_id sentence_id;
    char msg[NMEA_MAX_LENGTH + 1];

    for (int attempts = 0; attempts < 4; attempts++)
    {
        int i = 0;
        read(fd, msg, 1);
        while (msg[i] != '\n')
        {
            i++;
            read(fd, &msg[i], 1);
        }
        msg[++i] = '\0';

        sentence_id = minmea_sentence_id(msg, 0);
        if (sentence_id != MINMEA_INVALID && sentence_id != MINMEA_UNKNOWN)
        {
            return true;
        }
    }

    return false;
}

struct UbloxGPSData Gps::sampleImpl()
{
    Lock<FastMutex> l(mutex);
    return data;
}

void Gps::packSBASMessge(uint8_t* msg, int mode, int usage, int maxChannelNum,
                         int PRNs[3])
{
    int egnosPRNs[3] = {123, 126, 136};

    if (PRNs == NULL)
    {
        PRNs = egnosPRNs;
    }

    msg[0] = UbxHeader[0];
    msg[1] = UbxHeader[1];
    msg[2] = UbxCfgClass;
    msg[3] = UbxCfgId_SBAS;
    msg[4] = UbxCfgLen_SBAS & 0x00ff;
    msg[5] = (UbxCfgLen_SBAS & 0xff00) >> 8;

    msg[6] = mode;
    msg[7] = usage;
    msg[8] = maxChannelNum;

    for (int i = 0; i < 5; i++)
    {
        msg[9 + i] = 0x00;
    }

    for (int i = 0; i < 3; i++)
    {
        int bit =
            PRNs[i] - 120;  // 120 is the first valid PNR for SBAS satellites
        int byte = bit / 8;
        bit      = bit % 8;
        if (byte < 4)
        {
            msg[10 + byte] |= 1 << bit;
        }
        else
        {
            msg[9] |= 1 << bit;
        }
    }

    ubxChecksum(msg, SET_SBAS_MSG_LEN);
}

void Gps::packRateMessage(uint8_t* msg, int inbetweenReadings, int navRate,
                          TimeRef timeRef)
{
    uint16_t timeRefIdx = static_cast<uint16_t>(timeRef);

    msg[0] = UbxHeader[0];
    msg[1] = UbxHeader[1];
    msg[2] = UbxCfgClass;
    msg[3] = UbxCfgId_Rate;
    msg[4] = UbxCfgLen_Rate & 0x00ff;
    msg[5] = (UbxCfgLen_Rate & 0xff00) >> 8;

    msg[6] = inbetweenReadings & 0x00ff;
    msg[7] = (inbetweenReadings & 0xff00) >> 8;

    msg[8] = navRate & 0x00ff;
    msg[9] = (navRate & 0xff00) >> 8;

    msg[10] = timeRefIdx & 0x00ff;
    msg[11] = (timeRefIdx & 0xff00) >> 8;

    ubxChecksum(msg, SET_RATE_MSG_LEN);
}

void Gps::ubxChecksum(uint8_t* msg, int len)
{
    uint8_t ck_a = 0, ck_b = 0;

    for (int i = 2; i < len - 2;
         i++)  // checksum calculation from byte 2 to end of payload
    {
        ck_a = ck_a + msg[i];
        ck_b = ck_b + ck_a;
    }
    msg[len - 2] = ck_a;
    msg[len - 1] = ck_b;
}

void Gps::sendSBASMessage(int mode, int usage, int maxChannelNum, int PRNs[3])
{
    uint8_t msg[SET_SBAS_MSG_LEN];
    packSBASMessge(msg, mode, usage, maxChannelNum, PRNs);
    write(fd, msg, SET_SBAS_MSG_LEN);
}

void Gps::sendRateMessage(int inbetweenReadings, int navRate, TimeRef timeRef)
{
    uint8_t msg[SET_RATE_MSG_LEN];
    packRateMessage(msg, inbetweenReadings, navRate, timeRef);
    write(fd, msg, SET_RATE_MSG_LEN);
}
