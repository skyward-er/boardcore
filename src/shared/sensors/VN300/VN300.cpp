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

#include "VN300.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{
VN300::VN300(USARTType *portNumber, USARTInterface::Baudrate baudRate,
             CRCOptions crc, uint16_t samplePeriod)
    : portNumber(portNumber), baudRate(baudRate), crc(crc)
{
    this->samplePeriod = samplePeriod;
}

bool VN300::init()
{
    SensorErrors backup = lastError;

    // If already initialized
    if (isInit)
    {
        lastError = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor vn300 already initilized");
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
        LOG_ERR(logger, "Unable to initialize the receive vn300 string");
        return false;
    }

    if (!configDefaultSerialPort())
    {
        LOG_ERR(logger, "Unable to config the default vn100 serial port");
        return false;
    }

    if (!setCrc(false))
    {
        LOG_ERR(logger, "Unable to set the vn300 user selected CRC");
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

    if (!this->start())
    {
        LOG_ERR(logger, "Unable to start the sampling thread");
        return false;
    }

    // Set the isInit flag true
    isInit = true;

    // All good i restore the actual last error
    lastError = backup;

    return true;
}

void VN300::run()
{
    while (!shouldStop())
    {
        long long initialTime = miosix::getTick();

        // Sample the data locking the mutex
        miosix::Lock<FastMutex> l(mutex);
        threadSample = sampleData();

        // Sleep for the sampling period
        miosix::Thread::sleepUntil(initialTime + samplePeriod);
    }
}

VN300Data VN300::sampleData()
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

bool VN300::disableAsyncMessages(bool waitResponse)
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

bool VN300::AsyncPauseCommand(bool waitResponse, bool selection)
{
    if (selection)
    {
        if (!sendStringCommand(ASYNC_PAUSE_COMMAND))
        {
            return false;
        }
    }
    else
    {
        if (!sendStringCommand(ASYNC_RESUME_COMMAND))
        {
            return false;
        }
    }

    // Read the answer
    if (waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    return true;
}

bool VN300::configDefaultSerialPort()
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
bool VN300::configUserSerialPort()
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

bool VN300::setCrc(bool waitResponse)
{
    // Command for the crc change
    std::string command;
    CRCOptions backup = crc;

    // Check what type of crc is selected
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        // The 3 inside the command is the 16bit select. The others are default
        // values
        command = "VNRRG,30,0,0,0,0,3,0,1";
    }
    else
    {
        // Even if the CRC is not enabled i put the 8 bit
        // checksum because i need to know how many 'X' add at the end
        // of every command sent
        command = "VNRRG,30,0,0,0,0,1,0,1";
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

bool VN300::sendStringCommand(std::string command)
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

bool VN300::verifyChecksum(char *command, int length)
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

uint8_t VN300::calculateChecksum8(uint8_t *message, int length)
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

uint16_t VN300::calculateChecksum16(uint8_t *message, int length)
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