/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Lorenzo Cucchi, Fabrizio Monti
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

#include "VNCommonSerial.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/Debug.h>

namespace Boardcore
{

VNCommonSerial::VNCommonSerial(USART& usart, int baudrate,
                               const char* sensorName, CRCOptions crc,
                               const std::chrono::milliseconds timeout)
    : usart(usart), baudRate(baudrate), crc(crc),
      logger(Logging::getLogger(sensorName)), maxTimeout(timeout)
{
}

VNCommonSerial::~VNCommonSerial() {}

uint8_t VNCommonSerial::calculateChecksum8(const uint8_t* message, int length)
{
    int i;
    uint8_t result = 0x00;

    // Iterate and XOR all of the elements
    for (i = 0; i < length; i++)
    {
        //^ = XOR Operation
        result ^= message[i];
    }

    return result;
}

uint16_t VNCommonSerial::calculateChecksum16(const uint8_t* message, int length)
{
    int i;
    uint16_t result = 0x0000;

    // Apply the datasheet definition of CRC16-CCITT
    for (i = 0; i < length; i++)
    {
        result = (uint8_t)(result >> 8) | (result << 8);
        result ^= message[i];
        result ^= (uint8_t)(result & 0xff) >> 4;
        result ^= result << 12;
        result ^= (result & 0x00ff) << 5;
    }

    return result;
}

bool VNCommonSerial::verifyChecksum(char* command, int length)
{
    int checksumOffset = 0;

    // I look for the checksum position
    while (checksumOffset < length && command[checksumOffset] != '*')
        checksumOffset++;

    if (checksumOffset == length)
    {
        // The command doesn't have any checksum
        TRACE("No checksum in the command!\n");
        return false;
    }

    // Check based on the user selected crc type
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        if (length != checksumOffset + 5)  // 4 hex chars + 1 of position
        {
            TRACE("16 bit Checksum wrong length: %d != %d --> %s\n", length,
                  checksumOffset + 5, command);
            return false;
        }

        // Calculate the checksum and verify (comparison between numerical
        // checksum to avoid string bugs e.g 0856 != 865)
        if (strtol(command + checksumOffset + 1, NULL, 16) !=
            calculateChecksum16((uint8_t*)(command + 1), checksumOffset - 1))
        {
            TRACE("Different checksum: %s\n", command);
            return false;
        }
    }
    else if (crc == CRCOptions::CRC_ENABLE_8)
    {
        if (length != checksumOffset + 3)  // 2 hex chars + 1 of position
        {
            TRACE("8 bit Checksum wrong length: %d != %d --> %s\n", length,
                  checksumOffset + 3, command);
            return false;
        }

        // Calculate the checksum and verify (comparison between numerical
        // checksum to avoid string bugs e.g 0856 != 865)
        if (strtol(command + checksumOffset + 1, NULL, 16) !=
            calculateChecksum8((uint8_t*)(command + 1), checksumOffset - 1))
        {
            TRACE("Different checksum: %s\n", command);
            return false;
        }
    }

    return true;
}

bool VNCommonSerial::disableAsyncMessages(bool waitResponse)
{
    // Command string
    std::string command = "VNWRG,06,00";

    // Clear the receiving queue
    usart.clearQueue();

    if (!sendStringCommand(command))
        return false;

    if (waitResponse)
    {
        if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
            return false;

        if (checkErrorVN(recvString.data()))
            return false;
    }

    return true;
}

bool VNCommonSerial::setCrc(bool waitResponse)
{
    // Command for the crc change
    std::string command;
    CRCOptions backup = crc;

    // Check what type of crc is selected
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        // The 3 inside the command is the 16bit select. The others are default
        // values
        command = "VNWRG,30,0,0,0,0,3,0,1";
    }
    else
    {
        // Even if the CRC is not enabled i put the 8 bit
        // checksum because i need to know how many 'X' add at the end
        // of every command sent
        command = "VNWRG,30,0,0,0,0,1,0,1";
    }

    // I need to send the command in both crc because i don't know what type
    // of crc is previously selected. So in order to get the command accepted
    // i need to do it two times with different crc.
    crc = CRCOptions::CRC_ENABLE_8;

    // Send the command
    if (!sendStringCommand(command))
        return false;

    // Read the answer
    if (waitResponse)
    {
        if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
            return false;
    }

    crc = CRCOptions::CRC_ENABLE_16;

    // Send the command
    if (!sendStringCommand(command))
        return false;

    // Read the answer
    if (waitResponse)
    {
        if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
            return false;
    }

    // Restore the crc
    crc = backup;

    return true;
}

void VNCommonSerial::configDefaultSerialPort()
{
    // Initial default settings
    usart.setBaudrate(DEFAULT_BAUDRATE);
}

bool VNCommonSerial::configUserSerialPort()
{
    // I format the command to change baud rate
    std::string command = fmt::format("{}{}", "VNWRG,5,", baudRate);

    // I can send the command
    if (!sendStringCommand(command))
        return false;

    // I can open the serial with user's baud rate
    usart.setBaudrate(baudRate);

    // Check correct serial init
    return true;
}

bool VNCommonSerial::verifyModelNumber(const char* expectedModelNumber)
{
    // The model number starts from the 10th position
    const int modelNumberOffset = 10;

    // Clear the receiving queue
    usart.clearQueue();

    if (!sendStringCommand("VNRRG,01"))
    {
        LOG_WARN(logger, "Unable to send string command");
        return false;
    }

    miosix::Thread::sleep(100);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to receive string command");
        return false;
    }

    if (strncmp(expectedModelNumber, recvString.data() + modelNumberOffset,
                strlen(expectedModelNumber)) != 0)
    {
        LOG_ERR(logger, "Model number not corresponding: {} != {}",
                recvString.data(), expectedModelNumber);
        return false;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger, "Checksum verification failed: {}", recvString.data());
        return false;
    }

    return true;
}

QuaternionData VNCommonSerial::sampleQuaternion()
{
    unsigned int indexStart = 0;
    char* nextNumber;
    QuaternionData data;

    // Look for the second ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 2; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
            indexStart++;
        indexStart++;
    }

    // Parse the data
    data.quaternionTimestamp = TimestampTimer::getTimestamp();
    data.quaternionX = strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.quaternionY = strtod(nextNumber + 1, &nextNumber);
    data.quaternionZ = strtod(nextNumber + 1, &nextNumber);
    data.quaternionW = strtod(nextNumber + 1, NULL);

    return data;
}

MagnetometerData VNCommonSerial::sampleMagnetometer()
{
    unsigned int indexStart = 0;
    char* nextNumber;
    MagnetometerData data;

    // Look for the sixth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 6; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
            indexStart++;
        indexStart++;
    }

    // Parse the data
    data.magneticFieldTimestamp = TimestampTimer::getTimestamp();
    data.magneticFieldX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.magneticFieldY = strtod(nextNumber + 1, &nextNumber);
    data.magneticFieldZ = strtod(nextNumber + 1, NULL);

    return data;
}

AccelerometerData VNCommonSerial::sampleAccelerometer()
{
    unsigned int indexStart = 0;
    char* nextNumber;
    AccelerometerData data;

    // Look for the ninth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 9; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
            indexStart++;
        indexStart++;
    }

    // Parse the data
    data.accelerationTimestamp = TimestampTimer::getTimestamp();
    data.accelerationX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.accelerationY = strtod(nextNumber + 1, &nextNumber);
    data.accelerationZ = strtod(nextNumber + 1, NULL);

    return data;
}

GyroscopeData VNCommonSerial::sampleGyroscope()
{
    unsigned int indexStart = 0;
    char* nextNumber;
    GyroscopeData data;

    // Look for the twelfth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 12; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
            indexStart++;
        indexStart++;
    }

    // Parse the data
    data.angularSpeedTimestamp = TimestampTimer::getTimestamp();
    data.angularSpeedX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.angularSpeedY = strtod(nextNumber + 1, &nextNumber);
    data.angularSpeedZ = strtod(nextNumber + 1, NULL);

    return data;
}

uint8_t VNCommonSerial::checkErrorVN(const char* message)
{
    if (strncmp(message, "$VNERR,", 7) == 0)
    {
        // Extract the error code
        int errorCode = atoi(&message[7]);
        return errorCode;  // Error detected
    }

    return 0;  // No error detected
}

bool VNCommonSerial::closeAndReset()
{
    // Send the reset command to the VN300
    if (!sendStringCommand("VNRST"))
    {
        LOG_WARN(logger, "Impossible to reset the VN300");
        return false;
    }

    isInit = false;

    return true;
}

bool VNCommonSerial::startHSIEstimator(uint8_t convergeRate)
{
    D(assert((convergeRate >= 1 && convergeRate <= 5) &&
             "convergeRate must be between 1 and 5"));

    // Select HSI_RUN, runs the real-time hard/soft
    // iron calibration.
    const uint8_t hsiMode = 1;

    // Select USE_ONBOARD, Onboard HSI is applied to
    // the magnetic measurements.
    const uint8_t hsiOutput = 3;

    std::string command =
        fmt::format("VNWRG,44,{},{},{}", hsiMode, hsiOutput, convergeRate);

    return writeRegister(command);
}

bool VNCommonSerial::stopHSIEstimator()
{
    // Select HSI_OFF, real-time hard/soft iron calibration
    // algorithm is turned off.
    const uint8_t hsiMode = 0;

    // Keep USE_ONBOARD, Onboard HSI is applied to
    // the magnetic measurements.
    const uint8_t hsiOutput = 3;

    // Not influent, we are stopping the algorithm
    const uint8_t convergeRate = 5;

    std::string command =
        fmt::format("VNWRG,44,{},{},{}", hsiMode, hsiOutput, convergeRate);

    return writeRegister(command);
}

std::string VNCommonSerial::getHSIEstimatorValues()
{
    // Clear the receiving queue
    usart.clearQueue();

    if (!sendStringCommand("VNRRG,47"))
    {
        LOG_ERR(logger, "getHSIEstimatorValues: unable to send string command");
        return "";
    }

    miosix::Thread::sleep(100);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger,
                 "getHSIEstimatorValues: unable to receive string command");
        return "";
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger,
                "getHSIEstimatorValues: checksum verification failed: {}",
                recvString.data());
        return "";
    }

    return recvString.data();
}

bool VNCommonSerial::setMagnetometerCompensation(const Eigen::Matrix3f& c,
                                                 const Eigen::Vector3f& b)
{
    std::string command =
        fmt::format("VNWRG,23,{},{},{},{},{},{},{},{},{},{},{},{}", c(0, 0),
                    c(0, 1), c(0, 2), c(1, 0), c(1, 1), c(1, 2), c(2, 0),
                    c(2, 1), c(2, 2), b(0), b(1), b(2));

    return writeRegister(command);
}

bool VNCommonSerial::setAccelerometerCompensation(const Eigen::Matrix3f& c,
                                                  const Eigen::Vector3f& b)
{
    std::string command =
        fmt::format("VNWRG,25,{},{},{},{},{},{},{},{},{},{},{},{}", c(0, 0),
                    c(0, 1), c(0, 2), c(1, 0), c(1, 1), c(1, 2), c(2, 0),
                    c(2, 1), c(2, 2), b(0), b(1), b(2));

    return writeRegister(command);
}

bool VNCommonSerial::setGyroscopeCompensation(const Eigen::Matrix3f& c,
                                              const Eigen::Vector3f& b)
{
    std::string command =
        fmt::format("VNWRG,84,{},{},{},{},{},{},{},{},{},{},{},{}", c(0, 0),
                    c(0, 1), c(0, 2), c(1, 0), c(1, 1), c(1, 2), c(2, 0),
                    c(2, 1), c(2, 2), b(0), b(1), b(2));

    return writeRegister(command);
}

bool VNCommonSerial::saveConfiguration()
{
    // Clear the receiving queue
    usart.clearQueue();

    if (!sendStringCommand("VNWNV"))
    {
        LOG_ERR(logger, "saveConfiguration: unable to send string command");
        return false;
    }

    // The write settings command takes ~500ms to complete
    miosix::Thread::sleep(500);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "saveConfiguration: unable to receive string command");
        return false;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger, "saveConfiguration: checksum verification failed: {}",
                recvString.data());
        return false;
    }

    return true;
}

bool VNCommonSerial::restoreFactorySettings()
{
    // Clear the receiving queue
    usart.clearQueue();

    if (!sendStringCommand("VNRFS"))
    {
        LOG_ERR(logger,
                "restoreFactorySettings: unable to send string command");
        return false;
    }

    miosix::Thread::sleep(100);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger,
                 "restoreFactorySettings: unable to receive string command");
        return false;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger,
                "restoreFactorySettings: checksum verification failed: {}",
                recvString.data());
        return false;
    }

    // Everything is fine, let the sensor restart
    miosix::Thread::sleep(2000);

    return true;
}

bool VNCommonSerial::sendStringCommand(std::string command)
{
    if (crc == CRCOptions::CRC_ENABLE_8)
    {
        char checksum[4];  // 2 hex + \n + \0
        // I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum8((uint8_t*)command.c_str(), command.length()),
             checksum, 16);
        checksum[2] = '\n';
        checksum[3] = '\0';
        // I concatenate
        command = fmt::format("{}{}{}{}", "$", command, "*", checksum);
    }
    else if (crc == CRCOptions::CRC_ENABLE_16)
    {
        char checksum[6];  // 4 hex + \n + \0
        // I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum16((uint8_t*)command.c_str(), command.length()),
             checksum, 16);
        checksum[4] = '\n';
        checksum[5] = '\0';
        // I concatenate
        command = fmt::format("{}{}{}{}", "$", command, "*", checksum);
    }
    else
    {
        // No checksum, i add only 'XX' at the end and not 'XXXX' because
        // in cas of CRC_NO the enabled crc is 8 bit
        command = fmt::format("{}{}{}", "$", command, "*XX\n");
    }

    // I send the final command
    usart.writeString(command.c_str());

    /**
     * Wait enough to let the message reach the sensor. The wait time is
     * expressed in milliseconds. The baudrate is expressed in bps.
     * As a safety measure 10ms are added to the wait time.
     *
     * Thus the command size is multiplied by 1000 to obtain milliseconds and by
     * 8 because the baudrate is expressed in bit per second.
     */
    float waitTime = command.size() * 8000;
    waitTime /= baudRate;
    waitTime = ceil(waitTime);
    waitTime += 10;
    miosix::Thread::sleep(waitTime);

    return true;
}

bool VNCommonSerial::recvStringCommand(char* command, int maxLength)
{
    int i = 0;
    // Read the buffer
    if (!usart.readBlocking(command, maxLength, maxTimeout))
        return false;

    // Iterate until i reach the end or i find \n then i substitute it with a \0
    while (i < maxLength && command[i] != '\n')
        i++;

    // Terminate the string
    command[i] = '\0';

    // Assing the length
    recvStringLength = i - 1;

    return true;
}

bool VNCommonSerial::writeRegister(const std::string& command)
{
    usart.clearQueue();
    if (!sendStringCommand(command))
    {
        LOG_ERR(logger, "Write register failed: could not send the command");
        return false;
    }

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_ERR(logger, "Write register failed: recvStringCommand() failed");
        return false;
    }

    if (checkErrorVN(recvString.data()))
    {
        LOG_ERR(logger, "Write register failed: {}", recvString.data());
        return false;
    }

    return true;
}

}  // namespace Boardcore
