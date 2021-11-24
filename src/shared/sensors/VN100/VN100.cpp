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


/**
 * CONSTRUCTORS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
VN100::VN100()
    : VN100(defaultPortNumber, defaultBaudRate)
{}

VN100::VN100(unsigned int portNumber, unsigned int baudRate)
    : VN100(portNumber, baudRate, CRC_ENABLE_8)
{}

VN100::VN100(unsigned int portNumber, unsigned int baudRate, uint8_t crc)
{
    this -> portNumber  = portNumber;
    this -> baudRate    = baudRate;
    this -> crc         = crc;
    //Zero the isInit
    isInit = false;
}


/**
 * PUBLIC METHODS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
bool VN100::init()
{
    //If already initialized
    if(isInit)
    {
        return true;
    }

    //Allocate the receive vector with a malloc
    //TODO review this operation
    //recvString = (char *) malloc(recvStringMaxDimension * sizeof(char));
    recvString = new char[recvStringMaxDimension];

    if(recvString == NULL)
    {
        return false;
    }

    if(!configDefaultSerialPort())
    {
        return false;
    }

    if(!setCrc(false))
    {
        return false;
    }

    if(!disableAsyncMessages(false))
    {
        return false;
    }

    if(!configUserSerialPort())
    {
        return false;
    }

    //I need to repeat this in case of a non default
    //serial port communication at the beginning
    if(!setCrc(true))
    {
        return false;
    }

    if(!disableAsyncMessages(true))
    {
        return false;
    }

    /*//Reopen to delete junk values
    if(!configUserSerialPort())
    {
        return false;
    }*/

    //Set the isInit flag true
    isInit = true;

    return true;
}


bool VN100::closeAndReset()
{
    //Send the reset command to the vn100
    if(!sendStringCommand("VNRST"))
    {
        return false;
    }

    //Close the serial
    if(!(serialInterface -> closeSerial()))
    {
        return false;
    }

    isInit = false;

    //Free the recvString memory
    delete(recvString);

    //Free the serialInterface memory
    delete(serialInterface);

    return true;
}

bool VN100::selfTest()
{
    if(!selfTestImpl())
    {
        return false;
    }
    return true;
}


/**
 * PRIVATE METHODS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
bool VN100::disableAsyncMessages(bool waitResponse)
{
    //Command string
    std::string command = "VNWRG,06,00";  //Put 0 in register number 6 (ASYNC Config)

    //Send the command
    if(!sendStringCommand(command))
    {
        return false;
    }

    //Read the answer
    if(waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    return true;
}

bool VN100::configDefaultSerialPort()
{
    //Initial default settings
    serialInterface = new VN100Serial(portNumber, defaultBaudRate);

    //Check correct serial init
    if(!(serialInterface -> init()))
    {
        return false;
    }

    return true;
}

/**
 * EVEN IF THE USER CONFIGURED BAUD RATE IS THE DEFAULT I WANT
 * TO RESET THE BUFFER TO CLEAN THE JUNK
 */
bool VN100::configUserSerialPort()
{
    std::string command;

    //I format the command to change baud rate
    command = fmt::format("{}{}", "VNWRG,5,", baudRate);
    
    //I can send the command
    if(!sendStringCommand(command))
    {
        return false;
    }

    //I can close the serial
    serialInterface -> closeSerial();

    //Destroy the serial object
    delete(serialInterface);

    //I can open the serial with user's baud rate
    serialInterface = new VN100Serial(portNumber, baudRate);

    //Check correct serial init
    if(!(serialInterface -> init()))
    {
        return false;
    }
    return true;
}

bool VN100::setCrc(bool waitResponse)
{
    //Command for the crc change
    std::string command;
    uint8_t backup = crc;

    //Check what type of crc is selected
    if(crc == CRC_ENABLE_16)
    {
        //The 3 inside the command is the 16bit select. The others are default values
        command = "VNWRG,30,0,0,0,0,3,0,1";
    }
    else
    {
        //Even if the CRC is not enabled i put the 8 bit
        //checksum because i need to know how many 'X' add at the end
        //of every command sent
        command = "VNWRG,30,0,0,0,0,1,0,1";
    }

    //I need to send the command in both crc because i don't know what type
    //of crc is previously selected. So in order to get the command accepted
    //i need to do it two times with different crc.
    crc = CRC_ENABLE_8;
    //Send the command
    if(!sendStringCommand(command))
    {
        return false;
    }

    //Read the answer
    if(waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    crc = CRC_ENABLE_16;
    //Send the command
    if(!sendStringCommand(command))
    {
        return false;
    }

    //Read the answer
    if(waitResponse)
    {
        recvStringCommand(recvString, recvStringMaxDimension);
    }

    //Restore the crc
    crc = backup;
    return true;
}

bool VN100::selfTestImpl()
{
    char modelNumber[] = "VN-100";
    const int modelNumberOffset = 10;

    //Check the init status
    if(!isInit)
    {
        return false;
    }

    //I check the model number
    if(!sendStringCommand("VNRRG,01"))
    {
        return false;
    }

    if(!recvStringCommand(recvString, recvStringMaxDimension))
    {
        return false;
    }

    //Now i check that the model number is VN-100 starting from the 10th position
    //because of the message structure 
    if(strncmp(modelNumber, recvString + modelNumberOffset, strlen(modelNumber)) != 0)
    {
        TRACE("VN-100 not corresponding: %s != %s\n", recvString, modelNumber);
        return false;
    }

    //I check the checksum
    if(!verifyChecksum(recvString, recvStringLength))
    {
        TRACE("Checksum verification failed: %s\n", recvString);
        return false;
    }

    return true;
}

VN100Data VN100::sampleImpl()
{
    if(!isInit)
    {
        TRACE("Unable to sample, sensor not initialized!");
        return last_sample;
    }

    //Returns Quaternion, Magnetometer, Accelerometer and Gyro
    if(!sendStringCommand("VNRRG,15"))
    {
        //If something goes wrong i return the last sampled data
        return last_sample;
    }

    if(!recvStringCommand(recvString, recvStringMaxDimension))
    {
        //If something goes wrong i return the last sampled data
        return last_sample;
    }

    if(!verifyChecksum(recvString, recvStringLength))
    {
        //If something goes wrong i return the last sampled data
        return last_sample;
    }

    //Now i have to parse the data
    QuaternionData quat     = sampleQuaternion();
    MagnetometerData mag    = sampleMagnetometer();
    AccelerometerData acc   = sampleAccelerometer();
    GyroscopeData gyro      = sampleGyroscope();
    TemperatureData temp    = sampleTemperature();
    PressureData press      = samplePressure();

    return VN100Data(0, quat, mag, acc, gyro, temp, press);
}

QuaternionData VN100::sampleQuaternion()
{
    unsigned int indexStart = 0;
    char * nextNumber;
    //Data result
    QuaternionData data;

    //Look for the second ',' in the string
    //I can avoid the string control because it has already been done in sampleImpl
    for(int i = 0; i < 2; i++)
    {
        while(indexStart < recvStringLength && recvString[indexStart] != ',') { indexStart++; }
        indexStart++;
    }

    //Parse the data
    data.quat_x = strtod(recvString + indexStart + 1, &nextNumber);
    data.quat_y = strtod(nextNumber + 1, &nextNumber);
    data.quat_z = strtod(nextNumber + 1, &nextNumber);
    data.quat_w = strtod(nextNumber + 1, NULL);

    return data;
}

MagnetometerData VN100::sampleMagnetometer()
{
    unsigned int indexStart = 0;
    char * nextNumber;
    //Data result
    MagnetometerData data;

    //Look for the sixth ',' in the string
    //I can avoid the string control because it has already been done in sampleImpl
    for(int i = 0; i < 6; i++)
    {
        while(indexStart < recvStringLength && recvString[indexStart] != ',') { indexStart++; }
        indexStart++;
    }

    //Parse the data
    data.mag_x = strtod(recvString + indexStart + 1, &nextNumber);
    data.mag_y = strtod(nextNumber + 1, &nextNumber);
    data.mag_z = strtod(nextNumber + 1, NULL);

    return data;
}

AccelerometerData VN100::sampleAccelerometer()
{
    unsigned int indexStart = 0;
    char * nextNumber;
    //Data result
    AccelerometerData data;

    //Look for the ninth ',' in the string
    //I can avoid the string control because it has already been done in sampleImpl
    for(int i = 0; i < 9; i++)
    {
        while(indexStart < recvStringLength && recvString[indexStart] != ',') { indexStart++; }
        indexStart++;
    }

    //Parse the data
    data.accel_x = strtod(recvString + indexStart + 1, &nextNumber);
    data.accel_y = strtod(nextNumber + 1, &nextNumber);
    data.accel_z = strtod(nextNumber + 1, NULL);

    return data;
}

GyroscopeData VN100::sampleGyroscope()
{
    unsigned int indexStart = 0;
    char * nextNumber;
    //Data result
    GyroscopeData data;

    //Look for the twelfth ',' in the string
    //I can avoid the string control because it has already been done in sampleImpl
    for(int i = 0; i < 12; i++)
    {
        while(indexStart < recvStringLength && recvString[indexStart] != ',') { indexStart++; }
        indexStart++;
    }

    //Parse the data
    data.gyro_x = strtod(recvString + indexStart + 1, &nextNumber);
    data.gyro_y = strtod(nextNumber + 1, &nextNumber);
    data.gyro_z = strtod(nextNumber + 1, NULL);

    return data;
}

TemperatureData VN100::sampleTemperature()
{
    TemperatureData data;
    return data;
}

PressureData VN100::samplePressure()
{
    PressureData data;
    return data;
}

bool VN100::sendStringCommand(std::string command)
{
    if(crc == CRC_ENABLE_8)
    {
        char checksum[4]; //2 hex + \n + \0
        //I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum8((uint8_t *)command.c_str(), command.length()), checksum, 16);
        checksum[2] = '\n';
        checksum[3] = '\0';
        //I concatenate
        command = fmt::format("{}{}{}{}", "$", command, "*", checksum);

    }
    else if(crc == CRC_ENABLE_16)
    {
        char checksum[6]; //4 hex + \n + \0
        //I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum16((uint8_t *)command.c_str(), command.length()), checksum, 16);
        checksum[4] = '\n';
        checksum[5] = '\0';
        //I concatenate
        command = fmt::format("{}{}{}{}", "$", command, "*", checksum);
    }
    else
    {
        //No checksum, i add only 'XX' at the end and not 'XXXX' because
        //in cas of CRC_NO the enabled crc is 8 bit
        command = fmt::format("{}{}{}", "$", command, "*XX\n");
    }

    //I send the final command
    if(!(serialInterface -> send(command.c_str(), command.length() + 1)))
    {
        return false;
    }

    //Wait some time
    miosix::Thread::sleep(10);

    return true;
}

bool VN100::recvStringCommand(char * command, int maxLength)
{
    int i = 0;
    //Read the buffer
    if(!(serialInterface -> recv(command, maxLength)))
    {
        return false;
    }

    //Iterate until i reach the end or i find \n then i substitute it with a \0
    while(i < maxLength && command[i] != '\n'){ i++; }

    //Terminate the string
    command[i] = '\0';

    //Assing the length
    recvStringLength = i - 1;

    return true;
}

bool VN100::verifyChecksum(char * command, int length)
{
    int checksumOffset = 0;

    //I look for the checksum position
    while(checksumOffset < length && command[checksumOffset] != '*') { checksumOffset++; }

    if(checksumOffset == length)
    {
        //The command doesn't have any checksum
        TRACE("No checksum in the command!\n");
        return false;
    }

    //Check based on the user selected crc type
    if(crc == CRC_ENABLE_16)
    {
        if(length != checksumOffset + 5) //4 hex chars + 1 of position
        {
            TRACE("16 bit Checksum wrong length: %d != %d --> %s\n", length, checksumOffset + 5, command);
            return false;
        }

        //Calculate the checksum and verify (comparison between numerical checksum to avoid string bugs e.g 0856 != 865)
        if(strtol(command + checksumOffset + 1, NULL, 16) != calculateChecksum16((uint8_t *) (command + 1), checksumOffset - 1))
        {
            TRACE("Different checksum: %s\n", command + checksumOffset + 1);
            return false;
        }
    }
    else if(crc == CRC_ENABLE_8)
    {
        if(length != checksumOffset + 3) //2 hex chars + 1 of position
        {
            TRACE("8 bit Checksum wrong length: %d != %d --> %s\n", length, checksumOffset + 3, command);
            return false;
        }

        //Calculate the checksum and verify (comparison between numerical checksum to avoid string bugs e.g 0856 != 865)
        if(strtol(command + checksumOffset + 1, NULL, 16) != calculateChecksum8((uint8_t *) (command + 1), checksumOffset - 1))
        {
            TRACE("Different checksum: %s\n", command + checksumOffset + 1);
            return false;
        }
    }

    return true;
}

uint8_t VN100::calculateChecksum8(uint8_t * message, int length)
{
    int i;
    uint8_t result = 0x00;

    //Iterate and XOR all of the elements
    for(i = 0; i < length; i++)
    {
        //^ = XOR Operation
        result ^= message[i];
    }

    return result;
}

uint16_t VN100::calculateChecksum16(uint8_t * message, int length)
{
    int i;
    uint16_t result = 0x0000;

    //Apply the datasheet definition of CRC16-CCITT
    for(i = 0; i < length; i++)
    {
        result  = (uint8_t)(result >> 8) | (result << 8);
        result ^= message[i];
        result ^= (uint8_t)(result & 0xff) >> 4;
        result ^= result << 12;
        result ^= (result & 0x00ff) << 5;
    }

    return result;
}