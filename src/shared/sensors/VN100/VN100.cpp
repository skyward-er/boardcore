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

#include "VN100.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

VN100::VN100(USARTType *portNumber, USARTInterface::Baudrate baudRate,
             CRCOptions crc)
    : portNumber(portNumber), baudRate(baudRate), crc(crc)
{
}

bool VN100::init()
{
    SensorErrors backup = lastError;

    // If already initialized
    if (isInit)
    {
        lastError = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor vn100 already initilized");
        return true;
    }

    // Allocate the receive vector
    recvString = new char[recvStringMaxDimension];

    // Allocate the pre loaded strings based on the user selected crc
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        preSampleImuString       = new string("$VNRRG,15*92EA\n");
        preSampleTempPressString = new string("$VNRRG,54*4E0F\n");
    }
    else
    {
        preSampleImuString       = new string("$VNRRG,15*77\n");
        preSampleTempPressString = new string("$VNRRG,54*72\n");
    }

    // Set the error to init fail and if the init process goes without problem
    // i restore it to the last error
    lastError = SensorErrors::INIT_FAIL;

    if (recvString == NULL)
    {
        LOG_ERR(logger, "Unable to initialize the receive vn100 string");
        return false;
    }

    if (!configDefaultSerialPort())
    {
        LOG_ERR(logger, "Unable to config the default vn100 serial port");
        return false;
    }

    if (!setCrc(false))
    {
        LOG_ERR(logger, "Unable to set the vn100 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(false))
    {
        LOG_ERR(logger, "Unable to disable async messages from vn100");
        return false;
    }

    if (!configUserSerialPort())
    {
        LOG_ERR(logger, "Unable to config the user vn100 serial port");
        return false;
    }

    // I need to repeat this in case of a non default
    // serial port communication at the beginning
    if (!setCrc(true))
    {
        LOG_ERR(logger, "Unable to set the vn100 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(true))
    {
        LOG_ERR(logger, "Unable to disable async messages from vn100");
        return false;
    }

    // Set the isInit flag true
    isInit = true;

    // All good i restore the actual last error
    lastError = backup;

    return true;
}

bool VN100::sampleRaw()
{
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger,
                 "Unable to sample due to not initialized vn100 sensor");
        return false;
    }

    // Send the IMU sampling command
    if (!(serialInterface->writeString(preSampleImuString->c_str())))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        return false;
    }

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    // Receive the string
    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        return false;
    }

    return true;
}

string VN100::getLastRawSample()
{
    // If not init i return the void string
    if (!isInit)
    {
        return string("");
    }

    return string(recvString, recvStringLength);
}

bool VN100::closeAndReset()
{
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor vn100 already not initilized");
        return true;
    }

    // Send the reset command to the vn100
    if (!sendStringCommand("VNRST"))
    {
        LOG_WARN(logger, "Impossible to reset the vn100");
        return false;
    }

    isInit = false;

    // Free the recvString memory
    delete recvString;

    // Free the serialInterface memory
    delete serialInterface;

    return true;
}

bool VN100::selfTest()
{
    if (!selfTestImpl())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        LOG_WARN(logger, "Unable to perform a successful vn100 self test");
        return false;
    }

    return true;
}

VN100Data VN100::sampleImpl()
{
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger,
                 "Unable to sample due to not initialized vn100 sensor");
        return lastSample;
    }

    // Before sampling i check for errors
    if (lastError != SensorErrors::NO_ERRORS)
    {
        return lastSample;
    }

    // Returns Quaternion, Magnetometer, Accelerometer and Gyro
    if (!(serialInterface->writeString(preSampleImuString->c_str())))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    if (!verifyChecksum(recvString, recvStringLength))
    {
        LOG_WARN(logger, "Vn100 sampling message invalid checksum");
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    // Now i have to parse the data
    QuaternionData quat   = sampleQuaternion();
    MagnetometerData mag  = sampleMagnetometer();
    AccelerometerData acc = sampleAccelerometer();
    GyroscopeData gyro    = sampleGyroscope();

    // Returns Magnetometer, Accelerometer, Gyroscope, Temperature and Pressure
    // (UNCOMPENSATED) DO NOT USE THESE MAGNETOMETER, ACCELEROMETER AND
    // GYROSCOPE VALUES
    if (!(serialInterface->writeString(preSampleTempPressString->c_str())))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    if (!verifyChecksum(recvString, recvStringLength))
    {
        LOG_WARN(logger, "Vn100 sampling message invalid checksum");
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    // Parse the data
    TemperatureData temp = sampleTemperature();
    PressureData press   = samplePressure();

    return VN100Data(quat, mag, acc, gyro, temp, press);
}

bool VN100::disableAsyncMessages(bool waitResponse)
{
    // Command string
    std::string command =
        "VNWRG,06,00";  // Put 0 in register number 6 (ASYNC Config)

    // Send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    // Read the answer
    if (waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    return true;
}

bool VN100::configDefaultSerialPort()
{
    // Initial default settings
    serialInterface = new USART(portNumber, USARTInterface::Baudrate::B115200);

    // Check correct serial init
    return serialInterface->init();
}

/**
 * Even if the user configured baudrate is the default, I want to reset the
 * buffer to clean the junk.
 */
bool VN100::configUserSerialPort()
{
    std::string command;

    // I format the command to change baud rate
    command = fmt::format("{}{}", "VNWRG,5,", static_cast<int>(baudRate));

    // I can send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    // Destroy the serial object
    delete serialInterface;

    // I can open the serial with user's baud rate
    serialInterface = new USART(portNumber, baudRate);

    // Check correct serial init
    return serialInterface->init();
}

bool VN100::setCrc(bool waitResponse)
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
    {
        return false;
    }

    // Read the answer
    if (waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    crc = CRCOptions::CRC_ENABLE_16;

    // Send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    // Read the answer
    if (waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    // Restore the crc
    crc = backup;

    return true;
}

bool VN100::selfTestImpl()
{
    char modelNumber[]          = "VN-100";
    const int modelNumberOffset = 10;

    // Check the init status
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(
            logger,
            "Unable to perform vn100 self test due to not initialized sensor");
        return false;
    }

    // I check the model number (I perform the procedure twice to delete junk
    // problems)
    sendStringCommand("VNRRG,01");
    miosix::Thread::sleep(
        100);  // These sleep are important at very high baud rates
    recvStringCommand(recvString, recvStringMaxDimension);
    miosix::Thread::sleep(100);

    if (!sendStringCommand("VNRRG,01"))
    {
        LOG_WARN(logger, "Unable to send string command");
        return false;
    }

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to receive string command");
        return false;
    }

    // Now i check that the model number is VN-100 starting from the 10th
    // position because of the message structure
    if (strncmp(modelNumber, recvString + modelNumberOffset,
                strlen(modelNumber)) != 0)
    {
        LOG_ERR(logger, "VN-100 not corresponding: {} != {}", recvString,
                modelNumber);
        return false;
    }

    // I check the checksum
    if (!verifyChecksum(recvString, recvStringLength))
    {
        LOG_ERR(logger, "Checksum verification failed: {}", recvString);
        return false;
    }

    return true;
}

QuaternionData VN100::sampleQuaternion()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    QuaternionData data;

    // Look for the second ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 2; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.quatTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.quatX         = strtod(recvString + indexStart + 1, &nextNumber);
    data.quatY         = strtod(nextNumber + 1, &nextNumber);
    data.quatZ         = strtod(nextNumber + 1, &nextNumber);
    data.quatW         = strtod(nextNumber + 1, NULL);

    return data;
}

MagnetometerData VN100::sampleMagnetometer()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    MagnetometerData data;

    // Look for the sixth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 6; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.magneticFieldTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.magneticFieldX = strtod(recvString + indexStart + 1, &nextNumber);
    data.magneticFieldY = strtod(nextNumber + 1, &nextNumber);
    data.magneticFieldZ = strtod(nextNumber + 1, NULL);

    return data;
}

AccelerometerData VN100::sampleAccelerometer()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    AccelerometerData data;

    // Look for the ninth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 9; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.accelerationTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.accelerationX = strtod(recvString + indexStart + 1, &nextNumber);
    data.accelerationY = strtod(nextNumber + 1, &nextNumber);
    data.accelerationZ = strtod(nextNumber + 1, NULL);

    return data;
}

GyroscopeData VN100::sampleGyroscope()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    GyroscopeData data;

    // Look for the twelfth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 12; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.angularVelocityTimestamp =
        TimestampTimer::getInstance().getTimestamp();
    data.angularVelocityX = strtod(recvString + indexStart + 1, &nextNumber);
    data.angularVelocityY = strtod(nextNumber + 1, &nextNumber);
    data.angularVelocityZ = strtod(nextNumber + 1, NULL);

    return data;
}

TemperatureData VN100::sampleTemperature()
{
    unsigned int indexStart = 0;
    TemperatureData data;

    // Look for the eleventh ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 11; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.temperatureTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.temperature          = strtod(recvString + indexStart + 1, NULL);

    return data;
}

PressureData VN100::samplePressure()
{
    unsigned int indexStart = 0;
    PressureData data;

    // Look for the twelfth ',' in the string
    // I can avoid the string control because it has already been done in
    // sampleImpl
    for (int i = 0; i < 12; i++)
    {
        while (indexStart < recvStringLength && recvString[indexStart] != ',')
        {
            indexStart++;
        }
        indexStart++;
    }

    // Parse the data
    data.pressureTimestamp = TimestampTimer::getInstance().getTimestamp();
    data.pressure          = strtod(recvString + indexStart + 1, NULL);

    return data;
}

bool VN100::sendStringCommand(std::string command)
{
    if (crc == CRCOptions::CRC_ENABLE_8)
    {
        char checksum[4];  // 2 hex + \n + \0
        // I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum8((uint8_t *)command.c_str(), command.length()),
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
        itoa(calculateChecksum16((uint8_t *)command.c_str(), command.length()),
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
    if (!serialInterface->writeString(command.c_str()))
    {
        return false;
    }

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(500);

    return true;
}

bool VN100::recvStringCommand(char *command, int maxLength)
{
    int i = 0;
    // Read the buffer
    if (!(serialInterface->read(command, maxLength)))
    {
        return false;
    }

    // Iterate until i reach the end or i find \n then i substitute it with a \0
    while (i < maxLength && command[i] != '\n')
    {
        i++;
    }

    // Terminate the string
    command[i] = '\0';

    // Assing the length
    recvStringLength = i - 1;

    return true;
}

bool VN100::verifyChecksum(char *command, int length)
{
    int checksumOffset = 0;

    // I look for the checksum position
    while (checksumOffset < length && command[checksumOffset] != '*')
    {
        checksumOffset++;
    }

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
            calculateChecksum16((uint8_t *)(command + 1), checksumOffset - 1))
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
            calculateChecksum8((uint8_t *)(command + 1), checksumOffset - 1))
        {
            TRACE("Different checksum: %s\n", command);
            return false;
        }
    }

    return true;
}

uint8_t VN100::calculateChecksum8(uint8_t *message, int length)
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

uint16_t VN100::calculateChecksum16(uint8_t *message, int length)
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

}  // namespace Boardcore
