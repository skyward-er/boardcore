/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi
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

/**
 * @brief Driver for the VN300 IMU.
 *
 * The VN-300 is a miniature, surface-mount, high-performance GPS-Aided Inertial
 * Navigation System (GPS/INS). Incorporating the latest solid-state MEMS sensor
 * technology, the VN-300 combines a set of 3-axis accelerometers, 3-axis gyros,
 * 3-axis magnetometer, a barometric pressure sensor, two separate 50-channel L1
 * GPS receivers, as well as a 32-bit processor into a miniature aluminum
 * enclosure.
 * The VN300 supports both binary and ASCII encoding for communication but via
 * serial and with the asynchronous mode disabled only ASCII is available. The
 * protocol also provides two algorithms to verify the integrity of the messages
 * (8 bit checksum and 16 bit CRC-CCITT) both selectable by the user using the
 * constructor method. The serial communication also can be established with
 * various baud rates:
 * - 9600
 * - 19200
 * - 38400
 * - 57600
 * - 115200
 * - 128000
 * - 230400
 * - 460800
 * - 921600
 */

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <fmt/format.h>
#include <sensors/Sensor.h>
#include <string.h>
#include <utils/Debug.h>

#include "VN300Data.h"
#include "drivers/usart/USART.h"

namespace Boardcore
{
/**
 * @brief Driver class for VN300
 */
class VN300 : public Sensor<VN300Data>, public ActiveObject
{
public:
    /**
     * @brief Checksum Options
     */
    enum class CRCOptions : uint8_t
    {
        CRC_NO        = 0x00,
        CRC_ENABLE_8  = 0x08,
        CRC_ENABLE_16 = 0x10
    };
    /**
     * @brief Async options
     * ASYNC_NO corresponds to no Async communication
     * ASYNC_P1 corresponds to Async comm only on serial port 1
     * ASYNC_P2 corresponds to Async comm only on serial port 2
     * ASYNC_BOTH both serial port are used
     */
    enum class AsyncOptions : uint16_t
    {
        ASYNC_NO   = 0x00,
        ASYNC_P1   = 0x01,
        ASYNC_P2   = 0x02,
        ASYNC_BOTH = 0x03
    };

    enum class RateDivisor : uint16_t
    {
        RDIV_400_HZ = 0x01,
        RDIV_200_HZ = 0x02,
        RDIV_100_HZ = 0x04,
        RDIV_50_HZ  = 0x08,
        RDIV_40_HZ  = 0x0A
    };

    /**
     * @brief Group's bit for binary register setup
     */
    enum OutputGroup : uint16_t
    {
        GROUP_1 = 1 << 0,
        GROUP_2 = 1 << 1,
        GROUP_3 = 1 << 2,
        GROUP_4 = 1 << 3,
        GROUP_5 = 1 << 4,
        GROUP_6 = 1 << 5,
        GROUP_7 = 1 << 6
    };

    /**
     * @brief Common Group
     */
    enum GroupField_1 : uint16_t
    {
        T_START     = 1 << 0,
        T_GPS       = 1 << 1,
        T_SYNC      = 1 << 2,
        Y_P_R       = 1 << 3,
        QUAT        = 1 << 4,
        ANG_RATE    = 1 << 5,
        POS         = 1 << 6,
        VEL         = 1 << 7,
        ACCEL       = 1 << 8,
        IMU         = 1 << 9,
        MAGPRES     = 1 << 10,
        DELT_TH     = 1 << 11,
        INS_STAT    = 1 << 12,
        SYNC_IN_CNT = 1 << 13,
        T_GPS_PPS   = 1 << 14
    };

    /**
     * @brief TIME Group
     */
    enum GroupField_2 : uint16_t
    {
        T_START      = 1 << 0,
        T_GPS        = 1 << 1,
        GPS_TOW      = 1 << 2,
        GPS_WEEK     = 1 << 3,
        T_SYNC       = 1 << 4,
        T_GPS_PPS    = 1 << 5,
        T_UTC        = 1 << 6,
        SYNC_IN_CNT  = 1 << 7,
        SYNC_OUT_CNT = 1 << 8,
        T_STATUS     = 1 << 9,
    };

    /**
     * @brief IMU Group
     */
    enum GroupField_3 : uint16_t
    {
        IMU_STAT  = 1 << 0,
        UNC_MAG   = 1 << 1,
        UNC_ACC   = 1 << 2,
        UNC_GYRO  = 1 << 3,
        TEMP      = 1 << 4,
        PRES      = 1 << 5,
        DELTA_TH  = 1 << 6,
        DELTA_VEL = 1 << 7,
        MAG       = 1 << 8,
        ACCEL     = 1 << 9,
        ANG_RATE  = 1 << 10,
    };

    /**
     * @brief GNSS1 Group
     */
    enum GroupField_4 : uint16_t
    {
        UTC       = 1 << 0,
        TOW       = 1 << 1,
        WEEK      = 1 << 2,
        N_SATS    = 1 << 3,
        FIX       = 1 << 4,
        POS_LLA   = 1 << 5,
        POS_ECEF  = 1 << 6,
        VEL_NED   = 1 << 7,
        VEL_ECEF  = 1 << 8,
        POS_U     = 1 << 9,
        VEL_U     = 1 << 10,
        TIME_U    = 1 << 11,
        TIME_INFO = 1 << 12,
        DOP       = 1 << 13,
        SAT_INFO  = 1 << 14,
        RAW_MEAS  = 1 << 15
    };

    /**
     * @brief Attitude Group
     */
    enum GroupField_5 : uint16_t
    {
        RESERVED    = 1 << 0,
        Y_P_R       = 1 << 1,
        QUAT        = 1 << 2,
        DCM         = 1 << 3,
        MAG_NED     = 1 << 4,
        ACC_NED     = 1 << 5,
        LIN_ACC_BOD = 1 << 6,
        LIN_ACC_NED = 1 << 7,
        YprU        = 1 << 8,
        RESERVED    = 1 << 9,
        RESERVED    = 1 << 10,
        RESERVED    = 1 << 11
    };

    /**
     * @brief INS Group
     */
    enum GroupField_6 : uint16_t
    {
        INS_STATUS   = 1 << 0,
        POS_LLA      = 1 << 1,
        POS_ECEF     = 1 << 2,
        VEL_BODY     = 1 << 3,
        VEL_NED      = 1 << 4,
        VEL_ECEF     = 1 << 5,
        MAG_ECEF     = 1 << 6,
        ACC_ECEF     = 1 << 7,
        LIN_ACC_ECEF = 1 << 8,
        POS_U        = 1 << 9,
        VEL_U        = 1 << 10
    };

    /**
     * @brief GNSS2 Group
     */
    enum GroupField_7 : uint16_t
    {
        UTC       = 1 << 0,
        TOW       = 1 << 1,
        WEEK      = 1 << 2,
        N_SATS    = 1 << 3,
        FIX       = 1 << 4,
        POS_LLA   = 1 << 5,
        POS_ECEF  = 1 << 6,
        VEL_NED   = 1 << 7,
        VEL_ECEF  = 1 << 8,
        POS_U     = 1 << 9,
        VEL_U     = 1 << 10,
        TIME_U    = 1 << 11,
        TIME_INFO = 1 << 12,
        DOP       = 1 << 13,
        SAT_INFO  = 1 << 14,
        RAW_MEAS  = 1 << 15
    };

    VN300(USARTType *portNumber    = USART1,
          USART::Baudrate baudRate = USART::Baudrate::B921600,
          CRCOptions crc           = CRCOptions::CRC_ENABLE_8,
          uint16_t samplePeriod    = 50);

    bool init() override;

    bool sampleRaw();

    bool closeAndReset();

    bool selfTest() override;

private:
    /**
     * @brief Active object method, about the thread execution
     */
    void run() override;

    /**
     * @brief Sampling method used by the thread
     *
     * @return VN100Data The sampled data
     */
    VN300Data sampleData();

    /**
     * @brief Disables the async messages that the vn100 is default configured
     * to send at 40Hz on startup.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool disableAsyncMessages(bool waitResponse = true);

    /**
     * @brief Pause async messages in order to write command without receiving
     * continuous data transmission
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @param selection if true pause the async output, if false restart the
     * output
     *
     * @return True if operation succeeded.
     *
     */
    bool VN300::AsyncPauseCommand(bool waitResponse = false, bool selection);

    /**
     * @brief Configures the default serial communication.
     *
     * @return True if operation succeeded.
     */
    bool configDefaultSerialPort();

    /**
     * @brief Configures the user defined serial communication.
     *
     * @return True if operation succeeded.
     */
    bool configUserSerialPort();

    /**
     * @brief Sets the user selected crc method.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool setCrc(bool waitResponse = true);

    /**
     * @brief Method implementation of self test.
     *
     * @return True if operation succeeded.
     */
    bool selfTestImpl();

    /**
     * @brief Sends the command to the sensor with the correct checksum added
     * so '*' symbol is not needed at the end of the string as well as the '$'
     * at the beginning of the command.
     *
     * @param command Command to send.
     *
     * @return True if operation succeeded.
     */
    bool sendStringCommand(std::string command);

    /**
     * @brief Receives a command from the VN100 serialInterface->recv() but
     * swaps the first \n with a \0 to close the message.
     *
     * @param command The char array which will be filled with the command.
     * @param maxLength Maximum length for the command array.
     *
     * @return True if operation succeeded.
     */
    bool recvStringCommand(char *command, int maxLength);

    /**
     * @brief Method to verify the crc validity of a command.
     *
     * @param command The char array which contains the command.
     * @param maxLength Maximum length for the command array.
     *
     * @return True if operation succeeded.
     */
    bool verifyChecksum(char *command, int maxLength);

    /**
     * @brief Calculate the 8bit checksum on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 8 bit checksum.
     */
    uint8_t calculateChecksum8(uint8_t *message, int length);

    /**
     * @brief Calculate the 16bit array on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 16 bit CRC16-CCITT error check.
     */
    uint16_t calculateChecksum16(uint8_t *message, int length);

    USARTType *portNumber;
    USART::Baudrate baudRate;
    uint16_t samplePeriod;
    CRCOptions crc;
    bool isInit = false;

    std::string ASYNC_PAUSE_COMMAND  = "VNASY,0";
    std::string ASYNC_RESUME_COMMAND = "VNASY,1";
    /**
     * @brief Pointer to the received string by the sensor. Allocated 1 time
     * only (200 bytes).
     */
    char *recvString = nullptr;

    /**
     * @brief Actual strlen() of the recvString.
     */
    unsigned int recvStringLength = 0;

    /**
     * @brief Serial interface that is needed to communicate
     * with the sensor via ASCII codes.
     */
    USARTInterface *serialInterface = nullptr;

    /**
     * @brief Mutex to synchronize the reading and writing of the threadSample
     */
    mutable miosix::FastMutex mutex;
    VN300Data threadSample;

    PrintLogger logger = Logging::getLogger("vn300");

    static const unsigned int recvStringMaxDimension = 200;
};
}  // namespace Boardcore