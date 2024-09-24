/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Lorenzo Cucchi, Fabrizio Monti
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

#include <diagnostic/PrintLogger.h>
#include <drivers/usart/USART.h>
#include <sensors/SensorData.h>

namespace Boardcore
{

class VNCommonSerial
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
     * @param BaudRate Selectd baudrate.
     * @param sensorName The name of the sensor (VN100/VN300/...).
     * @param crc Checksum option.
     * @param timeout The maximum time that will be waited when reading from the
     * sensor.
     */
    VNCommonSerial(USART &usart, int baudrate, const char *sensorName,
                   CRCOptions crc, const std::chrono::milliseconds timeout);

    ~VNCommonSerial();

    /**
     * @brief Method to reset the sensor to default values and to close
     * the connection. Used if you need to close and re initialize the sensor.
     *
     * @return True if operation succeeded.
     */
    bool closeAndReset();

protected:
    /**
     * @brief Calculate the 8bit checksum on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 8 bit checksum.
     */
    uint8_t calculateChecksum8(const uint8_t *message, int length);

    /**
     * @brief Calculate the 16bit array on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 16 bit CRC16-CCITT error check.
     */
    uint16_t calculateChecksum16(const uint8_t *message, int length);

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
     * @brief Disables the async messages that the sensor is default configured
     * to send at 40Hz on startup.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool disableAsyncMessages(bool waitResponse);

    /**
     * @brief Sets the user selected crc method.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool setCrc(bool waitResponse = true);

    /**
     * @brief Configures the default serial communication.
     */
    void configDefaultSerialPort();

    /**
     * @brief Configures the user defined serial communication.
     *
     * @return True if operation succeeded.
     */
    bool configUserSerialPort();

    /**
     * @brief Verify the model number of the sensor.
     *
     * @param expectedModelNumber The expected model number.
     *
     *  @return True if the model number received from the sensor corresponds
     * with the expected one.
     */
    bool verifyModelNumber(const char *expectedModelNumber);

    /**
     * @brief Utility function used to retrieve the binary output from the
     * sensor.
     *
     * @param binaryData The variable that will hold the data.
     * @param sampleCommand The command to be sent to sample data.
     * @return True if operation successful, false otherwise.
     */
    template <typename T>
    bool getBinaryOutput(T &binaryData, const char *const sampleCommand);

    /**
     * @brief Utility function used to extract quaternion data from the
     * receiving string.
     */
    QuaternionData sampleQuaternion();

    /**
     * @brief Utility function used to extract magnetometer data from the
     * receiving string.
     */
    MagnetometerData sampleMagnetometer();

    /**
     * @brief Utility function used to extract accelerometer data from the
     * receiving string.
     */
    AccelerometerData sampleAccelerometer();

    /**
     * @brief Utility function used to extract gyroscope data from the receiving
     * string.
     */
    GyroscopeData sampleGyroscope();

    /**
     * @brief Check if the message received from the sensor contains an error.
     *
     * @param message The message to be checked.
     *
     * @return Returns 0 if no error was found, else returns the actual error
     * code.
     */
    uint8_t checkErrorVN(const char *message);

    /**
     * @brief Sends the command to the sensor with the correct checksum added
     * so '*' symbol is not needed at the end of the string as well as the '$'
     * at the beginning of the command. This function takes into account the
     * time needed for the command to reach the sensor. DO NOT USE THIS FUNCTION
     * IF LOW EXECUTION TIME IS NEEDED (for example when sending the sample
     * command).
     *
     * @param command Command to send.
     *
     * @return True if operation succeeded.
     */
    bool sendStringCommand(std::string command);

    /**
     * @brief Receives a command from the sensor but
     * swaps the first \n with a \0 to close the message.
     *
     * @param command The char array which will be filled with the command.
     * @param maxLength Maximum length for the command array.
     *
     * @return True if operation succeeded.
     */
    bool recvStringCommand(char *command, int maxLength);

    /**
     * @brief Serial interface that is needed to communicate
     * with the sensor via ASCII codes.
     */
    USART &usart;
    int baudRate;

    CRCOptions crc;

    PrintLogger logger;

    /**
     * @brief Default baudrate value for the usart communication.
     */
    static const int DEFAULT_BAUDRATE = 115200;

    /**
     * @brief Maximum size of the receiving string.
     */
    static const uint8_t recvStringMaxDimension = 200;

    /**
     * @brief Buffer used to store the string received from the sensor.
     */
    std::array<char, recvStringMaxDimension> recvString;

    /**
     * @brief Actual strlen() of the recvString.
     */
    uint8_t recvStringLength = 0;

    bool isInit = false;

private:
    /**
     * @brief The maximum time that will be waited during
     * a read from the serial channel.
     */
    const std::chrono::milliseconds maxTimeout;
};

template <typename T>
bool VNCommonSerial::getBinaryOutput(T &binaryData,
                                     const char *const sampleCommand)
{
    uint8_t initByte = 0;

    // Remove any junk
    usart.clearQueue();

    usart.writeString(sampleCommand);

    // Check the read of the 0xFA byte to find the start of the message
    if (usart.readBlocking(&initByte, 1, maxTimeout) && initByte == 0xFA)
    {
        if (usart.readBlocking(&binaryData, sizeof(T), maxTimeout))
        {
            return true;
        }
    }

    return false;
}

}  // namespace Boardcore
