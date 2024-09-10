/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi, Fabrizio Monti
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
#include <utils/Debug.h>

namespace Boardcore
{

VN300::VN300(USART& usart, int userBaudRate,
             VN300Defs::SampleOptions sampleOption, CRCOptions crc,
             const VN300Defs::AntennaPosition antPosA,
             const VN300Defs::AntennaPosition antPosB,
             const Eigen::Matrix3f rotMat)
    : VNCommonSerial(usart, userBaudRate, "VN300", crc),
      sampleOption(sampleOption), antPosA(antPosA), antPosB(antPosB),
      rotMat(rotMat)
{
}

bool VN300::init()
{
    SensorErrors backup = lastError;

    // If already initialized
    if (isInit)
    {
        lastError = SensorErrors::ALREADY_INIT;
        LOG_WARN(logger, "Sensor VN300 already initialized");
        return true;
    }

    // Allocate the pre loaded strings based on the user selected crc
    if (crc == CRCOptions::CRC_ENABLE_16)
    {
        preSampleBin1 = "$VNBOM,1*749D\n";
    }
    else
    {
        preSampleBin1 = "$VNBOM,1*45\n";
    }

    // Set the error to init fail and if the init process goes without problem
    // i restore it to the last error
    lastError = SensorErrors::INIT_FAIL;

    if (recvString.data() == NULL)
    {
        LOG_ERR(logger, "Unable to initialize the receive VN300 string");
        return false;
    }

    configDefaultSerialPort();

    if (!setCrc(false))
    {
        LOG_ERR(logger, "Unable to set the vn300 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(false))
    {
        LOG_ERR(logger, "Unable to disable async messages from vn300");
        return false;
    }

    if (!configUserSerialPort())
    {
        LOG_ERR(logger, "Unable to config the user vn300 serial port");
        return false;
    }

    if (!setBinaryOutput())
    {
        LOG_ERR(logger, "Unable to set binary output register");
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

    // I need to repeat this in case of a non default
    // serial port communication at the beginning
    if (!setCrc(true))
    {
        LOG_ERR(logger, "Unable to set the vn300 user selected CRC");
        return false;
    }

    if (!disableAsyncMessages(true))
    {
        LOG_ERR(logger, "Unable to disable async messages from vn300");
        return false;
    }

    // Set the isInit flag true
    isInit = true;

    // All good i restore the actual last error
    lastError = backup;

    return true;
}

bool VN300::selfTest() { return true; }

VN300Data VN300::sampleImpl()
{
    D(assert(isInit && "init() was not called"));

    // Reset any errors
    lastError = SensorErrors::NO_ERRORS;

    if (sampleOption == VN300Defs::SampleOptions::FULL)
    {
        return sampleFull();
    }
    else if (sampleOption == VN300Defs::SampleOptions::ARP)
    {
        return sampleArp();
    }
    else
    {
        // Sample option not implemented
        lastError = SensorErrors::COMMAND_FAILED;
        return lastSample;
    }
}

VN300Data VN300::sampleFull()
{
    VN300Data data;
    VN300Defs::BinaryDataFull bindataFull;

    const uint64_t timestamp = TimestampTimer::getTimestamp();

    bool sampleOutcome =
        false;  // True if a valid sample was retrieved from the sensor
    bool validChecksum = false;

    sampleOutcome =
        getBinaryOutput<VN300Defs::BinaryDataFull>(bindataFull, preSampleBin1);
    if (!sampleOutcome)
    {
        lastError = NO_NEW_DATA;
    }

    validChecksum =
        crc == CRCOptions::CRC_NO ||
        calculateChecksum16(reinterpret_cast<uint8_t*>(&bindataFull),
                            sizeof(bindataFull)) == 0;
    if (!validChecksum)
    {
        lastError = SensorErrors::BUS_FAULT;
    }

    // Verify if the sample is valid
    sampleOutcome = sampleOutcome && validChecksum;

    if (sampleOutcome)
    {
        buildBinaryDataFull(bindataFull, data, timestamp);
        return data;
    }
    else
    {
        // Last error is already set
        return lastSample;
    }
}

VN300Data VN300::sampleArp()
{
    VN300Data data;
    VN300Defs::BinaryDataArp binDataArp;

    const uint64_t timestamp = TimestampTimer::getTimestamp();

    bool sampleOutcome =
        false;  // True if a valid sample was retrieved from the sensor
    bool validChecksum = false;

    sampleOutcome =
        getBinaryOutput<VN300Defs::BinaryDataArp>(binDataArp, preSampleBin1);
    if (!sampleOutcome)
    {
        lastError = NO_NEW_DATA;
    }

    validChecksum = crc == CRCOptions::CRC_NO ||
                    calculateChecksum16(reinterpret_cast<uint8_t*>(&binDataArp),
                                        sizeof(binDataArp)) == 0;
    if (!validChecksum)
    {
        lastError = SensorErrors::BUS_FAULT;
    }

    // Verify if the sample is valid
    sampleOutcome = sampleOutcome && validChecksum;

    if (sampleOutcome)
    {
        buildBinaryDataArp(binDataArp, data, timestamp);
        return data;
    }
    else
    {
        // Last error is already set
        return lastSample;
    }
}

void VN300::buildBinaryDataFull(const VN300Defs::BinaryDataFull& rawData,
                                VN300Data& data, const uint64_t timestamp)
{
    // Quaternion
    data.quaternionTimestamp = timestamp;
    data.quaternionW         = rawData.quatW_bin;
    data.quaternionX         = rawData.quatX_bin;
    data.quaternionY         = rawData.quatY_bin;
    data.quaternionZ         = rawData.quatZ_bin;

    // Accelerometer
    data.accelerationTimestamp = timestamp;
    data.accelerationX         = rawData.accx;
    data.accelerationY         = rawData.accy;
    data.accelerationZ         = rawData.accz;

    // Magnetometer
    data.magneticFieldTimestamp = timestamp;
    data.magneticFieldX         = rawData.magx;
    data.magneticFieldY         = rawData.magy;
    data.magneticFieldZ         = rawData.magz;

    // Gyroscope
    data.angularSpeedTimestamp = timestamp;
    data.angularSpeedX         = rawData.angx;
    data.angularSpeedY         = rawData.angy;
    data.angularSpeedZ         = rawData.angz;

    // Gps
    data.insTimestamp = timestamp;
    data.fix_gps      = rawData.fix;
    data.fix_ins      = rawData.fix;
    data.status       = rawData.ins_status;
    data.yaw          = rawData.yaw_bin;
    data.pitch        = rawData.pitch_bin;
    data.roll         = rawData.roll_bin;
    data.latitude     = rawData.latitude_bin;
    data.longitude    = rawData.longitude_bin;
    data.altitude     = rawData.altitude_bin;
    data.nedVelX      = rawData.velx;
    data.nedVelY      = rawData.vely;
    data.nedVelZ      = rawData.velz;
}

void VN300::buildBinaryDataArp(const VN300Defs::BinaryDataArp& rawData,
                               VN300Data& data, const uint64_t timestamp)
{
    data = VN300Data();

    // Gps
    data.insTimestamp = timestamp;
    data.latitude     = rawData.latitude;
    data.longitude    = rawData.longitude;
    data.altitude     = rawData.altitude;
    data.fix_gps      = rawData.gpsFix;

    // Angular
    data.angularSpeedTimestamp = timestamp;
    data.angularSpeedX         = rawData.angularX;
    data.angularSpeedY         = rawData.angularY;
    data.angularSpeedZ         = rawData.angularZ;

    // Yaw pith roll
    data.yaw   = rawData.yaw;
    data.pitch = rawData.pitch;
    data.roll  = rawData.roll;

    // Quaternion
    data.quaternionTimestamp = timestamp;
    data.quaternionW         = rawData.quaternionW;
    data.quaternionX         = rawData.quaternionX;
    data.quaternionY         = rawData.quaternionY;
    data.quaternionZ         = rawData.quaternionZ;
}

bool VN300::setAntennaA(VN300Defs::AntennaPosition antPos)
{
    std::string command;

    command = fmt::format("{}{},{},{}", "VNWRG,57,", antPos.posX, antPos.posY,
                          antPos.posZ);

    usart.clearQueue();
    if (!sendStringCommand(command))
    {
        return false;
    }

    recvStringCommand(recvString.data(), recvStringMaxDimension);

    if (checkErrorVN(recvString.data()))
        return false;

    return true;
}

bool VN300::setCompassBaseline(VN300Defs::AntennaPosition antPos)
{
    std::string command;

    command = fmt::format("{}{},{},{},{},{},{}", "VNWRG,93,", antPos.posX,
                          antPos.posY, antPos.posZ, antPos.uncX, antPos.uncY,
                          antPos.uncZ);

    usart.clearQueue();
    if (!sendStringCommand(command))
    {
        return false;
    }

    recvStringCommand(recvString.data(), recvStringMaxDimension);

    if (checkErrorVN(recvString.data()))
        return false;

    return true;
}

bool VN300::setReferenceFrame(Eigen::Matrix3f rotMat)
{

    std::string command =
        fmt::format("{}{},{},{},{},{},{},{},{},{}", "VNWRG,26,", rotMat(0, 0),
                    rotMat(0, 1), rotMat(0, 2), rotMat(1, 0), rotMat(1, 1),
                    rotMat(1, 2), rotMat(2, 0), rotMat(2, 1), rotMat(2, 2));

    usart.clearQueue();
    if (!sendStringCommand(command))
    {
        return false;
    }

    recvStringCommand(recvString.data(), recvStringMaxDimension);

    if (checkErrorVN(recvString.data()))
        return false;

    return true;
}

bool VN300::setBinaryOutput()
{
    uint16_t outputGroup = 0, commonGroup = 0, gnssGroup = 0;

    switch (sampleOption)
    {
        case VN300Defs::SampleOptions::FULL:
            outputGroup =
                VN300Defs::BINARYGROUP_COMMON | VN300Defs::BINARYGROUP_GPS;

            commonGroup = VN300Defs::COMMONGROUP_YAWPITCHROLL |
                          VN300Defs::COMMONGROUP_QUATERNION |
                          VN300Defs::COMMONGROUP_ANGULARRATE |
                          VN300Defs::COMMONGROUP_VELOCITY |
                          VN300Defs::COMMONGROUP_ACCEL |
                          VN300Defs::COMMONGROUP_MAGPRES |
                          VN300Defs::COMMONGROUP_INSSTATUS;

            gnssGroup = VN300Defs::GPSGROUP_NUMSATS | VN300Defs::GPSGROUP_FIX |
                        VN300Defs::GPSGROUP_POSLLA;
            break;
        case VN300Defs::SampleOptions::ARP:
            outputGroup =
                VN300Defs::BINARYGROUP_COMMON | VN300Defs::BINARYGROUP_GPS;
            commonGroup = VN300Defs::COMMONGROUP_YAWPITCHROLL |
                          VN300Defs::COMMONGROUP_QUATERNION |
                          VN300Defs::COMMONGROUP_ANGULARRATE;
            gnssGroup = VN300Defs::GPSGROUP_FIX | VN300Defs::GPSGROUP_POSLLA;

            break;
    }

    // "0" means that no asynchronous message is active
    // "8" represents the divider at which the asynchronous message is sent with
    // respect to the max rate
    const char* const comp = "VNWRG,75,0,8";

    std::string command = fmt::format("{},{},{:x},{:x}", comp, outputGroup,
                                      commonGroup, gnssGroup);

    usart.clearQueue();

    if (!sendStringCommand(command))
    {
        return false;
    }

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
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
