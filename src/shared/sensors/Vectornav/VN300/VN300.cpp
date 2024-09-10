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

#include "diagnostic/CpuMeter/CpuMeter.h"

namespace Boardcore
{

VN300::VN300(USART& usart, int userBaudRate,
             VN300Defs::BinaryOutputPacket binaryOutputPacket, CRCOptions crc,
             const VN300Defs::AntennaPosition antPosA,
             const VN300Defs::AntennaPosition antPosB,
             const Eigen::Matrix3f rotMat)
    : VNCommonSerial(usart, userBaudRate, "VN300", crc),
      binaryOutputPacket(binaryOutputPacket), antPosA(antPosA),
      antPosB(antPosB), rotMat(rotMat)
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
    D(assert(isInit && "init() was not called"));
    // Reset any errors
    lastError = SensorErrors::NO_ERRORS;

    return sampleBinary();
}

VN300Data VN300::sampleBinary()
{
    VN300Data data;
    VN300Defs::BinaryDataFull bindataFull;
    VN300Defs::BinaryDataArp binDataArp;

    const uint64_t timestamp = TimestampTimer::getTimestamp();

    bool sampleOutcome = false;
    bool validChecksum = false;
    switch (binaryOutputPacket)
    {
        case VN300Defs::BinaryOutputPacket::FULL:
            sampleOutcome = getBinaryOutput<VN300Defs::BinaryDataFull>(
                bindataFull, preSampleBin1);

            validChecksum =
                crc == CRCOptions::CRC_NO ||
                calculateChecksum16(reinterpret_cast<uint8_t*>(&bindataFull),
                                    sizeof(bindataFull)) == 0;
            break;
        case VN300Defs::BinaryOutputPacket::ARP:
            sampleOutcome = getBinaryOutput<VN300Defs::BinaryDataArp>(
                binDataArp, preSampleBin1);

            validChecksum =
                crc == CRCOptions::CRC_NO ||
                calculateChecksum16(reinterpret_cast<uint8_t*>(&binDataArp),
                                    sizeof(binDataArp)) == 0;
            break;
    }

    sampleOutcome = sampleOutcome && validChecksum;

    if (sampleOutcome)
    {
        switch (binaryOutputPacket)
        {
            case VN300Defs::BinaryOutputPacket::FULL:
                buildBinaryDataFull(bindataFull, data, timestamp);
                break;
            case VN300Defs::BinaryOutputPacket::ARP:
                buildBinaryDataArp(binDataArp, data, timestamp);
                break;
        }
        return data;
    }
    else
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }
}

void VN300::buildBinaryDataFull(const VN300Defs::BinaryDataFull& rawData,
                                VN300Data& data, const uint64_t timestamp)
{
    QuaternionData quat{timestamp, rawData.quatW_bin, rawData.quatX_bin,
                        rawData.quatY_bin, rawData.quatZ_bin};

    AccelerometerData acc{timestamp, rawData.accx, rawData.accy, rawData.accz};

    MagnetometerData mag{timestamp, rawData.magx, rawData.magy, rawData.magz};

    GyroscopeData gyro{timestamp, rawData.angx, rawData.angy, rawData.angz};

    VN300Defs::Ins_Lla ins{
        timestamp,
        rawData.fix,
        rawData.fix,  // add function to extract ins_fix from ins_status
        rawData.ins_status,
        rawData.yaw_bin,
        rawData.pitch_bin,
        rawData.roll_bin,
        static_cast<float>(rawData.latitude_bin),
        static_cast<float>(rawData.longitude_bin),
        static_cast<float>(rawData.altitude_bin),
        rawData.velx,
        rawData.vely,
        rawData.velz};

    data = VN300Data(quat, mag, acc, gyro, ins);
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

    // Read the answer

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

    // Read the answer

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

    // Read the answer
    recvStringCommand(recvString.data(), recvStringMaxDimension);

    if (checkErrorVN(recvString.data()))
        return false;

    return true;
}

bool VN300::setBinaryOutput()
{
    // Here the output groups and the elements of each group are declared.
    // Reference to VN300Defs for all the possible groups and elements.
    // In order to change this elements it's mandatory to modify the struct
    // BinaryData, doing this is also necessary to modify the VN300Data struct,
    // this cause also that the sampleBinary function needs to be modified.
    // Another side effect it's that if a changed is done here it's highly
    // probable that the ascii version can't sample all the same data.

    uint16_t outputGroup = 0, commonGroup = 0, gnssGroup = 0;

    // if (binaryOutputPacket == )
    switch (binaryOutputPacket)
    {
        case VN300Defs::BinaryOutputPacket::FULL:
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
        case VN300Defs::BinaryOutputPacket::ARP:
            outputGroup =
                VN300Defs::BINARYGROUP_COMMON | VN300Defs::BINARYGROUP_GPS;
            commonGroup = VN300Defs::COMMONGROUP_YAWPITCHROLL |
                          VN300Defs::COMMONGROUP_QUATERNION |
                          VN300Defs::COMMONGROUP_ANGULARRATE;
            gnssGroup = VN300Defs::GPSGROUP_FIX | VN300Defs::GPSGROUP_POSLLA;

            break;
    }

    // The "comp" string it's created in order format the message and also
    // compare the reply. The fields represents the binary register "75", the
    // type of communication "0" means that no asynchronous message is active
    // and "8" represents the divider at which the asynchronous message is sent
    // respect to the max rate
    const char* comp = "VNWRG,75,0,8";

    // Using fmt::format it's possible to format the string also adding the
    // groups and their respective fields
    std::string command = fmt::format("{},{},{:x},{:x}", comp, outputGroup,
                                      commonGroup, gnssGroup);

    usart.clearQueue();

    // Send the command
    if (!sendStringCommand(command))
    {
        return false;
    }

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        return false;
    }

    // The reply is compared with the comp variable and in case of an error the
    // received message is passed to the logger
    if (checkErrorVN(recvString.data()) != 0)
    {
        LOG_WARN(logger, "The reply is wrong {}", recvString.data());
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

    miosix::Thread::sleep(100);

    // removing junk
    usart.clearQueue();

    // I check the model number
    if (!sendStringCommand("VNRRG,01"))
    {
        LOG_WARN(logger, "Unable to send string command");
        lastError = SELF_TEST_FAIL;
        return false;
    }

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to receive string command");
        lastError = SELF_TEST_FAIL;
        return false;
    }

    // Now i check that the model number is VN-100 starting from the 10th
    // position because of the message structure
    if (strncmp(modelNumber, recvString.data() + modelNumberOffset,
                strlen(modelNumber)) != 0)
    {
        LOG_ERR(logger, "VN-300 not corresponding: {} != {}", recvString.data(),
                modelNumber);
        lastError = SELF_TEST_FAIL;
        return false;
    }

    // I check the checksum
    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_ERR(logger, "Checksum verification failed: {}", recvString.data());
        lastError = SELF_TEST_FAIL;
        return false;
    }

    return true;
}

}  // namespace Boardcore
