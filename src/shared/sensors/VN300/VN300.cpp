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

VN300::VN300(USART &usart, int baudRate, CRCOptions crc, uint16_t samplePeriod,
             AntennaPosition antPosA, AntennaPosition antPosB,
             Eigen::Matrix3f rotMat)
    : usart(usart), baudRate(baudRate), samplePeriod(samplePeriod), crc(crc),
      antPosA(antPosA), antPosB(antPosB), rotMat(rotMat)
{
}

bool VN300::init()
{
    SensorErrors backup = lastError;

    // If already initialized
    if (isInit)
    {
        lastError = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor VN300 already initilized");
        return true;
    }

    // Allocate the receive vector
    recvString = new char[recvStringMaxDimension];

    // Allocate the pre loaded strings based on the user selected crc
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        preSampleImuString = new string("$VNRRG,15*92EA\n");
        preSampleINSlla    = new string("$VNRRG,63*6BBB\n");
    }
    else
    {
        preSampleImuString = new string("$VNRRG,15*77\n");
        preSampleINSlla    = new string("$VNRRG,63*76\n");
    }

    // Set the error to init fail and if the init process goes without problem
    // i restore it to the last error
    lastError = SensorErrors::INIT_FAIL;

    if (recvString == NULL)
    {
        LOG_ERR(logger, "Unable to initialize the receive VN300 string");
        return false;
    }

    if (!findBaudrate())
    {
        LOG_ERR(logger, "Unable to find the VN300 baudrate");
        return false;
    }

    if (!asyncPause())
    {
        LOG_ERR(logger, "Unable to pause the async messages");
        return false;
    }

    // if (!configDefaultSerialPort())
    //{
    //     LOG_ERR(logger, "Unable to config the default VN300 serial port");
    //     return false;
    // }

    // if (!resetFactorySettings())
    //{
    //     LOG_ERR(logger, "Unable to reset the VN300 to factory settings");
    //     return false;
    // }

    if (!setCrc(false))
    {
        LOG_ERR(logger, "Unable to set the VN300 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(false))
    {
        LOG_ERR(logger, "Unable to disable async messages from VN300");
        return false;
    }

    if (!setAntennaA(antPosA))
    {
        LOG_ERR(logger, "Unable to set antenna A position");
        return false;
    }

    if (!setCompassBaseline(antPosB))
    {
        LOG_ERR(logger, "Unable to set compass baseline");
        return false;
    }

    if (!setReferenceFrame(rotMat))
    {
        LOG_ERR(logger, "Unable to set reference frame rotation");
        return false;
    }

    if (!writeSettingsCommand())
    {
        LOG_ERR(logger, "Unable to save settings to non-volatile memory");
        return false;
    }

    if (!configUserSerialPort())
    {
        LOG_ERR(logger, "Unable to config the user VN300 serial port");
        return false;
    }

    // I need to repeat this in case of a non default
    // serial port communication at the beginning
    if (!setCrc(true))
    {
        LOG_ERR(logger, "Unable to set the VN300 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(true))
    {
        LOG_ERR(logger, "Unable to disable async messages from VN300");
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
        {
            // Sample the data locking the mutex
            miosix::Lock<FastMutex> l(mutex);
            threadSample = sampleData();
        }
        // Sleep for the sampling period
        miosix::Thread::sleepUntil(initialTime + samplePeriod);
    }
}

bool VN300::sampleRaw()
{
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger,
                 "Unable to sample due to not initialized VN300 sensor");
        return false;
    }

    // Send the IMU sampling command
    usart.writeString(preSampleImuString->c_str());

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

string VN300::getLastRawSample()
{
    // If not init i return the void string
    if (!isInit)
    {
        return string("");
    }

    return string(recvString, recvStringLength);
}

bool VN300::closeAndReset()
{
    // Sensor not init
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger, "Sensor VN300 already not initilized");
        return true;
    }

    // Send the reset command to the VN300
    if (!sendStringCommand("VNRST"))
    {
        LOG_WARN(logger, "Impossible to reset the VN300");
        return false;
    }

    isInit = false;

    // Free the recvString memory
    delete recvString;

    return true;
}

bool VN300::writeSettingsCommand()
{
    if (!sendStringCommand("VNWNV"))
    {
        LOG_WARN(logger, "Impossible to save settings");
    }

    // Write settings command takes approximately 500ms
    miosix::Thread::sleep(500);

    // Send the reset command to the VN300 in order to restart the Kalman filter
    if (!sendStringCommand("VNRST"))
    {
        LOG_WARN(logger, "Impossible to reset the VN300");

        return false;
    }

    miosix::Thread::sleep(500);

    return true;
}

bool VN300::selfTest()
{
    if (!selfTestImpl())
    {
        lastError = SensorErrors::SELF_TEST_FAIL;
        LOG_WARN(logger, "Unable to perform a successful VN300 self test");
        return false;
    }

    return true;
}

VN300Data VN300::sampleImpl()
{
    miosix::Lock<FastMutex> l(mutex);
    return threadSample;
}

VN300Data VN300::sampleData()
{
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(logger,
                 "Unable to sample due to not initialized VN300 sensor");
        return lastSample;
    }

    // Before sampling i check for errors
    if (lastError != SensorErrors::NO_ERRORS)
    {
        return lastSample;
    }

    // Returns Quaternion, Magnetometer, Accelerometer and Gyro
    usart.writeString(preSampleImuString->c_str());

    // Wait some time

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        // If something goes wrong i return the last sampled data
        assert(false);
        return lastSample;
    }

    if (!verifyChecksum(recvString, recvStringLength))
    {
        LOG_WARN(logger, "VN300 sampling message invalid checksum");
        // If something goes wrong i return the last sampled data
        assert(false);
        return lastSample;
    }

    QuaternionData quat   = sampleQuaternion();
    MagnetometerData mag  = sampleMagnetometer();
    AccelerometerData acc = sampleAccelerometer();
    GyroscopeData gyro    = sampleGyroscope();

    // Returns INS LLA message
    usart.writeString(preSampleINSlla->c_str());

    // Wait some time

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        return lastSample;
    }

    if (!verifyChecksum(recvString, recvStringLength))
    {
        LOG_WARN(logger, "VN300 sampling message invalid checksum");
        assert(false);
        return lastSample;
    }

    Ins_Lla ins = sampleIns();

    return VN300Data(quat, mag, acc, gyro, ins);
}

bool VN300::asyncPause()
{
    std::string command = "$VNASY,0*XX\n\0";

    usart.writeString(command.c_str());

    miosix::Thread::sleep(1000);

    return true;
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

        if (checkErrorVN(recvString))
            return false;
    }

    return true;
}

bool VN300::configDefaultSerialPort()
{
    // Initial default settings
    usart.setBaudrate(115200);

    // Check correct serial init
    return true;
}

bool VN300::findBaudrate()
{
    for (uint32_t i = 0; i < BaudrateList.size(); i++)
    {
        // I set the baudrate
        usart.setBaudrate(BaudrateList[i]);
        printf("Baudrate: %d\n", BaudrateList[i]);
        char initChar;
        uint64_t in_time = miosix::getTick();
        while (miosix::getTick() - in_time < 30)
        {
            usart.writeString("$VNRRG,01*XX\n");
            // Read the first char
            if (usart.read(&initChar, 1))
            {
                // If it is a '$' i break
                if (initChar == '$')
                {
                    return true;
                }
            }
        }
    }

    // If i'm here, i didn't find the correct baudrate
    return false;
}

/**
 * Even if the user configured baudrate is the default, I want to reset the
 * buffer to clean the junk.
 */
bool VN300::configUserSerialPort()
{
    std::string command;

    // I format the command to change baud rate
    command = fmt::format("{}{}", "VNWRG,5,", baudRate);

    // I can send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    if (!recvStringCommand(recvString, recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        return false;
    }

    if (checkErrorVN(recvString))
    {
        LOG_WARN(logger, "Unable to change serial port baudrate");
        return false;
    }
    else
    {
        // I can open the serial with user's baud rate
        usart.setBaudrate(baudRate);
    }

    return true;
}

bool VN300::resetFactorySettings()
{
    // Command string
    std::string command =
        "VNRFS";  // Put 0 in register number 0 (Factory Settings)

    // Send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    // if (!recvStringCommand(recvString, recvStringMaxDimension))
    //{
    //     LOG_WARN(logger, "Unable to sample due to serial communication
    //     error"); return false;
    // }

    return true;
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
        checkErrorVN(recvString);
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
        checkErrorVN(recvString);
    }

    // Restore the crc
    crc = backup;

    return true;
}

bool VN300::setAntennaA(AntennaPosition antPos)
{
    std::string command;

    command = fmt::format("{}{},{},{}", "VNWRG,57,", antPos.posX, antPos.posY,
                          antPos.posZ);

    if (!sendStringCommand(command))
    {
        return false;
    }

    return true;
}

bool VN300::setCompassBaseline(AntennaPosition antPos)
{
    std::string command;

    command = fmt::format("{}{},{},{},{},{},{}", "VNWRG,93,", antPos.posX,
                          antPos.posY, antPos.posZ, antPos.uncX, antPos.uncY,
                          antPos.uncZ);

    if (!sendStringCommand(command))
    {
        return false;
    }

    return true;
}

bool VN300::setReferenceFrame(Eigen::Matrix3f rotMat)
{
    std::string command;

    command =
        fmt::format("{}{},{},{},{},{},{},{},{},{}", "VNWRG,26,", rotMat(0, 0),
                    rotMat(0, 1), rotMat(0, 2), rotMat(1, 0), rotMat(1, 1),
                    rotMat(1, 2), rotMat(2, 0), rotMat(2, 1), rotMat(2, 2));

    // I can send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    return true;
}

bool VN300::selfTestImpl()
{
    char modelNumber[]          = "VN-300T-CR";
    const int modelNumberOffset = 10;

    // Check the init status
    if (!isInit)
    {
        lastError = SensorErrors::NOT_INIT;
        LOG_WARN(
            logger,
            "Unable to perform VN300 self test due to not initialized sensor");
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
        LOG_ERR(logger, "VN-300 not corresponding: {} != {}", recvString,
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

QuaternionData VN300::sampleQuaternion()
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
    data.quatTimestamp = TimestampTimer::getTimestamp();
    data.quatX         = strtod(recvString + indexStart + 1, &nextNumber);
    data.quatY         = strtod(nextNumber + 1, &nextNumber);
    data.quatZ         = strtod(nextNumber + 1, &nextNumber);
    data.quatW         = strtod(nextNumber + 1, NULL);

    return data;
}

MagnetometerData VN300::sampleMagnetometer()
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
    data.magneticFieldTimestamp = TimestampTimer::getTimestamp();
    data.magneticFieldX = strtod(recvString + indexStart + 1, &nextNumber);
    data.magneticFieldY = strtod(nextNumber + 1, &nextNumber);
    data.magneticFieldZ = strtod(nextNumber + 1, NULL);

    return data;
}

AccelerometerData VN300::sampleAccelerometer()
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
    data.accelerationTimestamp = TimestampTimer::getTimestamp();
    data.accelerationX = strtod(recvString + indexStart + 1, &nextNumber);
    data.accelerationY = strtod(nextNumber + 1, &nextNumber);
    data.accelerationZ = strtod(nextNumber + 1, NULL);

    return data;
}

GyroscopeData VN300::sampleGyroscope()
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
    data.angularSpeedTimestamp = TimestampTimer::getTimestamp();
    data.angularSpeedX = strtod(recvString + indexStart + 1, &nextNumber);
    data.angularSpeedY = strtod(nextNumber + 1, &nextNumber);
    data.angularSpeedZ = strtod(nextNumber + 1, NULL);

    return data;
}

Ins_Lla VN300::sampleIns()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    Ins_Lla data;

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
    data.insTimestamp = TimestampTimer::getTimestamp();
    data.time_gps     = strtod(recvString + indexStart, &nextNumber);
    data.week = static_cast<uint16_t>(strtol(nextNumber + 1, &nextNumber, 16));
    data.status =
        static_cast<uint16_t>(strtol(nextNumber + 1, &nextNumber, 16));
    data.yaw       = strtof(nextNumber + 1, &nextNumber);
    data.pitch     = strtof(nextNumber + 1, &nextNumber);
    data.roll      = strtof(nextNumber + 1, &nextNumber);
    data.latitude  = strtof(nextNumber + 1, &nextNumber);
    data.longitude = strtof(nextNumber + 1, &nextNumber);
    data.altitude  = strtof(nextNumber + 1, &nextNumber);
    data.nedVelX   = strtof(nextNumber + 1, &nextNumber);
    data.nedVelY   = strtof(nextNumber + 1, &nextNumber);
    data.nedVelZ   = strtof(nextNumber + 1, NULL);

    return data;
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
    printf("%s\n", command.c_str());
    // I send the final command
    usart.writeString(command.c_str());

    return true;
}

bool VN300::recvStringCommand(char *command, int maxLength)
{
    uint64_t end_time = 0;
    uint64_t in_time  = TimestampTimer::getTimestamp();
    char initChar;
    for (;;)
    {
        // Read the first char
        if (!usart.readBlocking(&initChar, 1))
        {
            return false;
        }
        // If it is a '$' i break
        if (initChar == '$')
        {
            break;
        }
    }
    command[0] = '$';

    int i = 0;
    // Read the buffer

    if (!usart.readBlocking(command + 1, maxLength - 1))
    {
        return false;
    }

    // Iterate until i reach the end or i find \n then i substitute it with a \0
    while (i < maxLength && command[i] != '\n')
    {
        i++;
        // assert for testing purposes
        assert(i < maxLength);
    }

    // Terminate the string
    command[i] = '\0';

    // Assing the length
    recvStringLength = i - 1;

    end_time = TimestampTimer::getTimestamp() - in_time;
    printf("Time to read: %lld\n", end_time);
    
    return true;
}

bool VN300::checkErrorVN(const char *message)
{
    if (strncmp(message, "$VNERR,", 7) == 0)
    {
        // Extract the error code
        int errorCode = atoi(&message[7]);
        string error;
        // Handle the error based on the error code
        switch (errorCode)
        {
            case 1:
                error = "VN300 Hard Fault";
                break;
            case 2:
                error = "VN300 Serial Buffer Overflow";
                break;
            case 3:
                error = "VN300 Invalid Checksum";
                break;
            case 4:
                error = "VN300 Invalid Command";
                break;
            case 5:
                error = "VN300 Not Enough Parameters";
                break;
            case 6:
                error = "VN300 Too Many Parameters";
                break;
            case 7:
                error = "VN300 Invalid Parameter";
                break;
            case 8:
                error = "VN300 Invalid Register";
                break;
            case 9:
                error = "VN300 Unauthorized Access";
                break;
            case 10:
                error = "VN300 Watchdog Reset";
                break;
            case 11:
                error = "VN300 Output Buffer Overflow";
                break;
            case 12:
                error = "VN300 Insufficient Baud Rate";
                break;
            case 255:
                error = "VN300 Error Buffer Overflow";
                break;
            default:
                error = "VN300 Unknown error";
                break;
        }

        printf("%s\n", error.c_str());

        return true;  // Error detected
    }

    return false;  // No error detected
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
