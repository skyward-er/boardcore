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
#include <utils/KernelTime.h>

namespace Boardcore
{

VN100::VN100(USART &usart, int baudRate, CRCOptions crc, uint16_t samplePeriod)
    : VNCommonSerial(usart, baudRate, "VN100-serial", crc),
      samplePeriod(samplePeriod)
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

    if (recvString.data() == NULL)
    {
        LOG_ERR(logger, "Unable to initialize the receive vn100 string");
        return false;
    }

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

void VN100::run()
{
    while (!shouldStop())
    {
        long long initialTime = Kernel::getOldTick();
        {
            // Sample the data locking the mutex
            miosix::Lock<FastMutex> l(mutex);
            threadSample = sampleData();
        }
        // Sleep for the sampling period
        Kernel::Thread::sleepUntil(initialTime + samplePeriod);
    }
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
    usart.writeString(preSampleImuString->c_str());

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    // Receive the string
    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
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

    return string(recvString.data(), recvStringLength);
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
    miosix::Lock<FastMutex> l(mutex);
    return threadSample;
}

VN100Data VN100::sampleData()
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
    usart.writeString(preSampleImuString->c_str());

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
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
    usart.writeString(preSampleTempPressString->c_str());

    // Wait some time
    // TODO dimension the time
    miosix::Thread::sleep(1);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        // If something goes wrong i return the last sampled data
        return lastSample;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
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

    // removing junk
    usart.clearQueue();

    // I check the model number
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

    // Now i check that the model number is VN-100 starting from the 10th
    // position because of the message structure
    if (strncmp(modelNumber, recvString.data() + modelNumberOffset,
                strlen(modelNumber)) != 0)
    {
        LOG_ERR(logger, "VN-100 not corresponding: {} != {}", recvString.data(),
                modelNumber);
        return false;
    }

    // I check the checksum
    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger, "Checksum verification failed: {}", recvString.data());
        return false;
    }

    return true;
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
    data.temperatureTimestamp = TimestampTimer::getTimestamp();
    data.temperature = strtod(recvString.data() + indexStart + 1, NULL);

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
    data.pressureTimestamp = TimestampTimer::getTimestamp();
    data.pressure          = strtod(recvString.data() + indexStart + 1, NULL);

    return data;
}

}  // namespace Boardcore
