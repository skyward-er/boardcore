/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Fabrizio Monti
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
 * magnetometer, gyroscope, barometer and temperature sensor. The device
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

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <fmt/format.h>
#include <sensors/Sensor.h>
#include <string.h>
#include <utils/Debug.h>

#include "VN100Data.h"
#include "drivers/usart/USART.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 IMU.
 */
class VN100 : public Sensor<VN100Data>
{
public:
    enum class CRCOptions : uint8_t
    {
        CRC_NO        = 0x00,
        CRC_ENABLE_8  = 0x08,
        CRC_ENABLE_16 = 0x10
    };

    /**
     * @brief Constructor.
     *
     * @param usart Serial bus used for the sensor.
     * @param BaudRate different from the sensor's default [9600, 19200, 38400,
     * 57600, 115200, 128000, 230400, 460800, 921600].
     * @param crc Checksum options.
     */
    VN100(USART &usart, int baudrate, CRCOptions crc = CRCOptions::CRC_NO);

    bool init() override;

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
     * @brief Struct used to store the binary data received from the sensor.
     */
    struct __attribute__((packed)) BinaryData
    {
        uint8_t group;
        uint16_t group1;
        float quaternionX;
        float quaternionY;
        float quaternionZ;
        float quaternionW;
        float angularX;
        float angularY;
        float angularZ;
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        float magneticFieldX;
        float magneticFieldY;
        float magneticFieldZ;
        float temperature;
        float pressure;
        uint16_t crc;
    };

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
     * @brief This function sets the output binary mode for the sensor. It
     * configures the sensor in order to read acceleration, angular speed,
     * magnetic field, quaternion, temperature and pressure data.
     *
     * @return Returns true if successful, false otherwise.
     */
    bool setBinaryOutput();

    /**
     * @brief Method implementation of self test.
     *
     * @return True if operation succeeded.
     */
    bool selfTestImpl();

    /**
     * @brief Builds VN100Data starting from the binary data obtained by the
     * sensor (stored in `binData`).
     *
     * @param sampleTimestamp The timestamp corresponding to the sampled data.
     *
     * @return Returns the VN100Data object.
     */
    VN100Data buildBinaryData(const uint64_t sampleTimestamp);

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
    bool verifyChecksum(const char *command, const int maxLength);

    /**
     * @brief Calculate the 8bit checksum on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 8 bit checksum.
     */
    uint8_t calculateChecksum8(const uint8_t *message, const int length);

    /**
     * @brief Calculate the 16bit array on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 16 bit CRC16-CCITT error check.
     */
    uint16_t calculateChecksum16(const uint8_t *message, const int length);

    /**
     * @brief Serial interface that is needed to communicate
     * with the sensor via ASCII codes.
     */
    USART &usart;
    const int baudRate;

    CRCOptions crc;
    bool isInit = false;

    /**
     * @brief This variable contains the data received from the sample, before
     * being converted to VN100Data.
     */
    BinaryData binData;

    /**
     * @brief Max dimension of a message from the sensor.
     */
    static const uint8_t recvStringMaxDimension = 200;

    /**
     * @brief Pointer to the received string by the sensor. Allocated 1 time
     * only (200 bytes).
     */
    char recvString[recvStringMaxDimension] = "";

    /**
     * @brief Actual strlen() of the recvString.
     */
    uint8_t recvStringLength = 0;

    /**
     * @brief Pre computed command used to ask a binary sample to the sensor.
     */
    const char *askSampleCommand = "";

    PrintLogger logger = Logging::getLogger("vn100");
};
}  // namespace Boardcore
