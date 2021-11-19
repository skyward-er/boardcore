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

    if(!configDefaultSerialPort())
    {
        return false;
    }

    if(!setCrc())
    {
        return false;
    }

    if(!disableAsyncMessages())
    {
        return false;
    }

    if(!configUserSerialPort())
    {
        return false;
    }

    //Allocate the receive vector with a malloc
    //TODO review this operation
    recvString = (char *) malloc(recvStringMaxDimension * sizeof(char));

    if(recvString == NULL)
    {
        return false;
    }

    //Read the left junk only if the baud rate is the default
    //else there is a serial reset as last operation so there is no need
    //to clean the junk
    if(baudRate == defaultBaudRate && 
       !(serialInterface -> recv(recvString, recvStringMaxDimension)))
    {
        return false;
    }

    //Set the isInit flag true
    isInit = true;

    return true;
}

bool VN100::selfTest()
{
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

    if(!(serialInterface -> recv(recvString, recvStringMaxDimension)))
    {
        return false;
    }

    printf("%s\n", recvString);

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

    return true;
}


/**
 * PRIVATE METHODS
 * ---------------------------------------------------------------------------------------------------------------------------------------
 */
bool VN100::disableAsyncMessages()
{
    //Command string
    std::string command = "VNWRG,06,00";  //Put 0 in register number 6 (ASYNC Config)

    //Send the command
    if(!sendStringCommand(command))
    {
        return false;
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

bool VN100::configUserSerialPort()
{
    std::string command;

    //If the the baud rate is already the default i can avoid the procedure
    if(baudRate == defaultBaudRate)
    {
        return true;
    }

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

bool VN100::setCrc()
{
    //Command for the crc change
    std::string command;

    //Check what type of crc is selected
    if(crc == CRC_ENABLE_16)
    {
        //The 3 inside the command is the 16bit select. The others are default values
        command = "VNWRG,30,0,0,0,0,3,0,1";

        //Send the command
        if(!sendStringCommand(command))
        {
            return false;
        }
    }
    else
    {
        //Even if the CRC is not enabled i put the 8 bit
        //checksum because i need to know how many 'X' add at the end
        //of every command sent
        command = "VNWRG,30,0,0,0,0,1,0,1";

        //Send the command
        if(!sendStringCommand(command))
        {
            return false;
        }
    }
    return true;
}

bool VN100::sendStringCommand(std::string command)
{
    if(crc == CRC_ENABLE_8)
    {
        char checksum[3]; //2 hex + \0
        //I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum8((uint8_t *)command.c_str(), command.length()), checksum, 16);
        //I concatenate
        command = fmt::format("{}{}{}{}{}", "$", command, "*", checksum, "\n");

    }
    else if(crc == CRC_ENABLE_16)
    {
        char checksum[5]; //4 hex + \0
        //I convert the calculated checksum in hex using itoa
        itoa(calculateChecksum16((uint8_t *)command.c_str(), command.length()), checksum, 16);
        //I concatenate
        command = fmt::format("{}{}{}{}{}", "$", command, "*", checksum, "\n");
    }
    else
    {
        //No checksum, i add only 'XX' at the end and not 'XXXX' because
        //in cas of CRC_NO the enabled crc is 8 bit
        command = fmt::format("{}{}{}", "$", command, "*XX\n");
    }

    //I send the final command
    if(!(serialInterface -> send(command.c_str(), command.length())))
    {
        return false;
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

VN100Data VN100::sampleImpl()
{
    VN100Data dato{};

    return dato;
}