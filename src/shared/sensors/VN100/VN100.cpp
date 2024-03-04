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

#include "VN100.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

VN100::VN100(USART &usart, int baudRate, CRCOptions crc)
    : usart(usart), baudRate(baudRate), crc(crc), binData({})
{
}

bool VN100::init()
{
    SensorErrors backup = lastError;

    // If already initialized
    if (isInit)
    {
        lastError = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor vn100 already initialized");
        return true;
    }

    // Allocate the pre loaded strings based on the user selected crc
    switch (this->crc)
    {
        case CRCOptions::CRC_ENABLE_8:
            askSampleCommand = "$VNBOM,1*45\n";
            break;
        case CRCOptions::CRC_ENABLE_16:
            askSampleCommand = "$VNBOM,1*749D\n";
            break;
        case CRCOptions::CRC_NO:
            askSampleCommand = "$VNBOM,1*XX\n";
            break;
    }

    // Set the error to init fail and if the init process goes without problem
    // i restore it to the last error
    lastError = SensorErrors::INIT_FAIL;

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

    if (!setBinaryOutput())
    {
        LOG_ERR(logger, "Unable to set the vn100 binary output");
        return false;
    }

    // Set the isInit flag true
    isInit = true;

    // All good i restore the actual last error
    lastError = backup;

    return true;
}

bool VN100::setBinaryOutput()
{
    // This command samples quaternion data, accelerometer, angular rate,
    // magnetometer, temperature and pressure (with async messages disabled)
    const char *setBinarySampleCommand = "";
    switch (this->crc)
    {
        case CRCOptions::CRC_ENABLE_8:
            setBinarySampleCommand = "$VNWRG,75,0,16,01,0530*44\n";
            break;
        case CRCOptions::CRC_ENABLE_16:
            setBinarySampleCommand = "$VNWRG,75,0,16,01,0530*9CFA\n";
            break;
        case CRCOptions::CRC_NO:
            setBinarySampleCommand = "$VNWRG,75,0,16,01,0530*XX\n";
            break;
    }

    // Send the command
    usart.writeString(setBinarySampleCommand);

    // Get the response from the sensor
    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        LOG_WARN(logger, "Cannot set binary output");
        return false;
    }

    // Verify if the sensor replied with an error
    if (strncmp(recvString, errorString.c_str(), errorString.length()) == 0)
    {
        LOG_WARN(logger, "Cannot set binary output");
        return false;
    }

    // Evaluate wait time when sampling
    // It is multiplied by 8000: 1000 in order to obtain milliseconds, 8 because
    // the baudrate is in bit per second
    float fWaitTime = (strlen(askSampleCommand) + sizeof(BinaryData)) * 8000;
    fWaitTime /= baudRate;
    fWaitTime = ceil(fWaitTime);
    fWaitTime += 1;  // +1ms for safety
    sampleWaitTime = (int)fWaitTime;

    return true;
}

bool VN100::closeAndReset()
{
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor vn100 already not initialized");
        return true;
    }

    // Send the reset command to the vn100
    if (!sendStringCommand("VNRST"))
    {
        LOG_WARN(logger, "Impossible to reset the vn100");
        return false;
    }

    isInit = false;

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
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor vn100 not initialized");
        return VN100Data{};
    }

    usart.writeString(askSampleCommand);

    miosix::Thread::sleep(sampleWaitTime);

    // "wait logic": it might take some time to receive the answer
    // from the sensor
    // This timestamp is also used as timestamp for the sampled data
    const uint64_t initTime = TimestampTimer::getTimestamp();

    unsigned char initByte =
        0;  // used to look for the first byte of the answer
    while (TimestampTimer::getTimestamp() - initTime <=
           30)  // TODO: dimension the time
    {
        // The reply message starts with 0xFA
        if (usart.read(&initByte, 1) && initByte == 0xFA)
        {
            // Reading all the message directly into the struct
            if (usart.read(&binData, sizeof(BinaryData)))
            {
                if (crc != CRCOptions::CRC_NO &&
                    calculateChecksum16(reinterpret_cast<uint8_t *>(&binData),
                                        sizeof(BinaryData)) != 0)
                {
                    // Data received from the sensor has an invalid checksum
                    // Discard data
                    lastError = SensorErrors::BUS_FAULT;
                    return lastSample;
                }

                VN100Data vnData = buildBinaryData(initTime);
                lastError        = NO_ERRORS;
                return vnData;
            }
        }
    }

    lastError = NO_NEW_DATA;
    return lastSample;
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
    usart.setBaudrate(115200);

    // Check correct serial init
    return true;
}

/**
 * Even if the user configured baudrate is the default, I want to reset the
 * buffer to clean the junk.
 */
bool VN100::configUserSerialPort()
{
    std::string command;

    // I format the command to change baud rate
    command = fmt::format("{}{}", "VNWRG,5,", baudRate);

    // I can send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    // I can open the serial with user's baud rate
    usart.setBaudrate(baudRate);

    // Check correct serial init
    return true;
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
    const char modelNumber[]    = "VN-100";
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

    // removing junk
    usart.clearQueue();

    // I check the model number
    if (!sendStringCommand("VNRRG,01"))
    {
        LOG_WARN(logger, "Unable to send string command");
        return false;
    }

    miosix::Thread::sleep(100);

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

VN100Data VN100::buildBinaryData(const uint64_t sampleTimestamp)
{
    VN100Data vnData;

    // Acceleration data
    vnData.accelerationTimestamp = sampleTimestamp;
    vnData.accelerationX         = binData.accelerationX;
    vnData.accelerationY         = binData.accelerationY;
    vnData.accelerationZ         = binData.accelerationZ;

    // Angular speed data
    vnData.angularSpeedTimestamp = sampleTimestamp;
    vnData.angularSpeedX         = binData.angularX;
    vnData.angularSpeedY         = binData.angularY;
    vnData.angularSpeedZ         = binData.angularZ;

    // Magnetometer data
    vnData.magneticFieldTimestamp = sampleTimestamp;
    vnData.magneticFieldX         = binData.magneticFieldX;
    vnData.magneticFieldY         = binData.magneticFieldY;
    vnData.magneticFieldZ         = binData.magneticFieldZ;

    // Quaternion data
    vnData.quaternionTimestamp = sampleTimestamp;
    vnData.quaternionX         = binData.quaternionX;
    vnData.quaternionY         = binData.quaternionY;
    vnData.quaternionZ         = binData.quaternionZ;
    vnData.quaternionW         = binData.quaternionW;

    // Temperature data
    vnData.temperatureTimestamp = sampleTimestamp;
    vnData.temperature          = binData.temperature;

    // Pressure data
    vnData.pressureTimestamp = sampleTimestamp;
    vnData.pressure          = binData.pressure;

    return vnData;
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

bool VN100::recvStringCommand(char *command, int maxLength)
{
    int i = 0;
    // Read the buffer
    if (!usart.readBlocking(command, maxLength))
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

    // Assign the length
    recvStringLength = i - 1;

    return true;
}

bool VN100::verifyChecksum(const char *command, const int length)
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

uint8_t VN100::calculateChecksum8(const uint8_t *message, const int length)
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

uint16_t VN100::calculateChecksum16(const uint8_t *message, const int length)
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
