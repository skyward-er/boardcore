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

#include "VN100Serial.h"

#include <drivers/timer/TimestampTimer.h>

namespace Boardcore
{

VN100Serial::VN100Serial(USART& usart, int baud, CRCOptions crc,
                         std::chrono::milliseconds timeout)
    : VNCommonSerial(usart, baud, "vn100-serial", crc, timeout)
{
}

bool VN100Serial::init()
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

    configDefaultSerialPort();

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

    if (!verifyModelNumber("VN-100"))
    {
        LOG_ERR(logger, "Error, model number not corresponding");
        lastError = INVALID_WHOAMI;
        return false;
    }

    if (!setBinaryOutput())
    {
        LOG_ERR(logger, "Unable to set binary output register");
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

bool VN100Serial::selfTest() { return true; }

VN100SerialData VN100Serial::sampleImpl()
{
    VN100SerialData data;
    BinaryData binData;

    const uint64_t timestamp = TimestampTimer::getTimestamp();

    bool sampleOutcome =
        false;  // True if a valid sample was retrieved from the sensor

    sampleOutcome = getBinaryOutput<BinaryData>(binData, askSampleCommand);
    if (!sampleOutcome)
    {
        lastError = NO_NEW_DATA;
    }

    // With binary output the checksum is always calculated with checksum16
    bool validChecksum =
        (crc == CRCOptions::CRC_NO) ||
        (calculateChecksum16(reinterpret_cast<uint8_t*>(&binData),
                             sizeof(binData)) == 0);

    if (sampleOutcome && !validChecksum)
    {
        lastError = SensorErrors::BUS_FAULT;
    }

    // Verify if the sample is valid
    sampleOutcome = sampleOutcome && validChecksum;

    if (sampleOutcome)
    {
        buildBinaryData(binData, data, timestamp);
        return data;
    }
    else
    {
        // Last error is already set
        return lastSample;
    }
}

void VN100Serial::buildBinaryData(const BinaryData& binData,
                                  VN100SerialData& data,
                                  const uint64_t sampleTimestamp)
{
    // Acceleration data
    data.accelerationTimestamp = sampleTimestamp;
    data.accelerationX         = binData.accelerationX;
    data.accelerationY         = binData.accelerationY;
    data.accelerationZ         = binData.accelerationZ;

    // Angular speed data
    data.angularSpeedTimestamp = sampleTimestamp;
    data.angularSpeedX         = binData.angularX;
    data.angularSpeedY         = binData.angularY;
    data.angularSpeedZ         = binData.angularZ;

    // Magnetometer data
    data.magneticFieldTimestamp = sampleTimestamp;
    data.magneticFieldX         = binData.magneticFieldX;
    data.magneticFieldY         = binData.magneticFieldY;
    data.magneticFieldZ         = binData.magneticFieldZ;

    // Quaternion data
    data.quaternionTimestamp = sampleTimestamp;
    data.quaternionX         = binData.quaternionX;
    data.quaternionY         = binData.quaternionY;
    data.quaternionZ         = binData.quaternionZ;
    data.quaternionW         = binData.quaternionW;

    // Temperature data
    data.temperatureTimestamp = sampleTimestamp;
    data.temperature          = binData.temperature;

    // Pressure data
    data.pressureTimestamp = sampleTimestamp;
    data.pressure          = binData.pressure;
}

bool VN100Serial::setBinaryOutput()
{
    // This command samples quaternion data, accelerometer, angular rate,
    // magnetometer, temperature and pressure (with async messages disabled)
    const char* setBinarySampleCommand = "";
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

    usart.clearQueue();

    if (!sendStringCommand(setBinarySampleCommand))
    {
        LOG_WARN(logger,
                 "sendStringCommand() failed, cannot set binary output");
        return false;
    }

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger,
                 "recvStringCommand() failed, cannot set binary output");
        return false;
    }

    if (checkErrorVN(recvString.data()) != 0)
    {
        LOG_WARN(logger, "Error while setting binary output: {}",
                 recvString.data());
        return false;
    }

    return true;
}

}  // namespace Boardcore
