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

#include "VNCommonSerial.h"

#include <drivers/timer/TimestampTimer.h>
#include <utils/Debug.h>

namespace Boardcore
{

VNCommonSerial::VNCommonSerial(USART &usart, int baudrate,
                               const std::string &sensorName, CRCOptions crc)
    : sensorName(sensorName), usart(usart), baudRate(baudrate), crc(crc),
      logger(Logging::getLogger(sensorName))
{
}

VNCommonSerial::~VNCommonSerial() {}

uint8_t VNCommonSerial::calculateChecksum8(const uint8_t *message, int length)
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

uint16_t VNCommonSerial::calculateChecksum16(const uint8_t *message, int length)
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

bool VNCommonSerial::verifyChecksum(char *command, int length)
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

void VNCommonSerial::clearBuffer()
{
    char c;
    while (usart.read(&c, 1))
    {
    }
}

bool VNCommonSerial::asyncPause()
{
    usart.writeString("$VNASY,0*XX\n");
    return true;
}

QuaternionData VNCommonSerial::sampleQuaternion()
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
    data.magneticFieldX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.magneticFieldY = strtod(nextNumber + 1, &nextNumber);
    data.magneticFieldZ = strtod(nextNumber + 1, NULL);

    return data;
}

AccelerometerData VNCommonSerial::sampleAccelerometer()
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
    data.accelerationX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.accelerationY = strtod(nextNumber + 1, &nextNumber);
    data.accelerationZ = strtod(nextNumber + 1, NULL);

    return data;
}

GyroscopeData VNCommonSerial::sampleGyroscope()
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
    data.angularSpeedX =
        strtod(recvString.data() + indexStart + 1, &nextNumber);
    data.angularSpeedY = strtod(nextNumber + 1, &nextNumber);
    data.angularSpeedZ = strtod(nextNumber + 1, NULL);

    return data;
}

}  // namespace Boardcore
