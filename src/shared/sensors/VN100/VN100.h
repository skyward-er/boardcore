/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
 * @brief Driver for the VN100S IMU.
 *
 * The VN100S sensor is a calibrated IMU which includes accelerometer,
 * magnetometer, gyroscope, barometer and temperature sensor. ùThe device
 * provides also a calibration matrix and an anti-drift matrix for the gyroscope
 * values. The goal of this driver though is to interface the sensor in its
 * basic use. Things like asynchronous data and anti-drift techniques haven't
 * been implemented yet. The driver is intended to be used with the "Rugged
 * sensor" version (aka only UART communication) although the actual VN100S chip
 * is capable also of SPI communication.
 *
 * The VN100S supports both binary and ASCII encoding for communication but via
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

#include <diagnostic/PrintLogger.h>
#include <fmt/format.h>
#include <sensors/Sensor.h>
#include <string.h>
#include <utils/Debug.h>

#include "VN100Data.h"
#include "VN100Serial.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 IMU.
 */
class VN100 : public Sensor<VN100Data>
{
public:
    enum class BaudRates : unsigned int
    {
        Baud_9600   = 9600,
        Baud_19200  = 19200,
        Baud_38400  = 38400,
        Baud_57600  = 57600,
        Baud_115200 = 115200,
        Baud_128000 = 128000,
        Baud_230400 = 230400,
        Baud_460800 = 460800,
        Baud_921600 = 921600
    };

    enum class CRCOptions : uint8_t
    {
        CRC_NO        = 0x00,
        CRC_ENABLE_8  = 0x08,
        CRC_ENABLE_16 = 0x10
    };

    /**
     * @brief Constructor.
     *
     * @param USART port number.
     * @param BaudRate different from the sensor's default.
     * @param Redundancy check option.
     */
    VN100(unsigned int portNumber = defaultPortNumber,
          BaudRates baudRate      = BaudRates::Baud_115200,
          CRCOptions crc          = CRCOptions::CRC_ENABLE_8);

    bool init() override;

    /**
     * @brief Method to sample the raw data without parsing.
     *
     * @return True if operation succeeded.
     */
    bool sampleRaw();

    /**
     * @brief Method to get the raw sample.
     *
     * @return String that represents the sample.
     */
    string getLastRawSample();

    /**
     * @brief Method to reset the sensor to default values and to close
     * the connection. Used if you need to close and re initialize the sensor.
     *
     * @return True if operation succeeded.
     */
    bool closeAndReset();

    bool selfTest() override;

private:
    /**
     * @brief Sample action implementation.
     */
    VN100Data sampleImpl() override;

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

    QuaternionData sampleQuaternion();

    MagnetometerData sampleMagnetometer();

    AccelerometerData sampleAccelerometer();

    GyroscopeData sampleGyroscope();

    TemperatureData sampleTemperature();

    PressureData samplePressure();

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

    unsigned int portNumber;
    BaudRates baudRate;
    CRCOptions crc;
    bool isInit = false;

    /**
     * @brief IMU pre-elaborated sample string for efficiency reasons.
     */
    string *preSampleImuString = nullptr;

    /**
     * @brief Temperature and pressure pre-elaborated sample string for
     * efficiency reasons.
     */
    string *preSampleTempPressString = nullptr;

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
    VN100Serial *serialInterface = nullptr;

    PrintLogger logger = Logging::getLogger("vn100");

    static const unsigned int defaultPortNumber      = 2;
    static const unsigned int recvStringMaxDimension = 200;
};
}  // namespace Boardcore
