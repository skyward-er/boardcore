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

VN300::VN300(USART &usart, int userBaudRate,
             VN300Defs::SamplingMethod samplingMethod, CRCOptions crc,
             const VN300Defs::AntennaPosition antPosA,
             const VN300Defs::AntennaPosition antPosB,
             const Eigen::Matrix3f rotMat)
    : VNCommonSerial(usart, userBaudRate, "VN300", crc),
      samplingMethod(samplingMethod), antPosA(antPosA), antPosB(antPosB),
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
        preSampleImuString = "$VNRRG,15*92EA\n";
        preSampleINSlla    = "$VNRRG,63*6BBB\n";
        preSampleBin1      = "$VNBOM,1*749D\n";
    }
    else
    {
        preSampleImuString = "$VNRRG,15*77\n";
        preSampleINSlla    = "$VNRRG,63*76\n";
        preSampleBin1      = "$VNBOM,1*45\n";
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

    if (samplingMethod == VN300Defs::SamplingMethod::BINARY)
    {
        return sampleBinary();
    }
    else
    {
        return sampleASCII();
    }
}

VN300Data VN300::sampleBinary()
{
    VN300Defs::BinaryData bindata;

    const uint64_t timestamp = TimestampTimer::getTimestamp();

    if (getBinaryOutput<VN300Defs::BinaryData>(bindata, preSampleBin1))
    {
        QuaternionData quat{timestamp, bindata.quatW_bin, bindata.quatX_bin,
                            bindata.quatY_bin, bindata.quatZ_bin};

        AccelerometerData acc{timestamp, bindata.accx, bindata.accy,
                              bindata.accz};

        MagnetometerData mag{timestamp, bindata.magx, bindata.magy,
                             bindata.magz};

        GyroscopeData gyro{timestamp, bindata.angx, bindata.angy, bindata.angz};

        VN300Defs::Ins_Lla ins{
            timestamp,
            bindata.fix,
            bindata.fix,  // add function to extract ins_fix from ins_status
            bindata.ins_status,
            bindata.yaw_bin,
            bindata.pitch_bin,
            bindata.roll_bin,
            static_cast<float>(bindata.latitude_bin),
            static_cast<float>(bindata.longitude_bin),
            static_cast<float>(bindata.altitude_bin),
            bindata.velx,
            bindata.vely,
            bindata.velz};

        return VN300Data(quat, mag, acc, gyro, ins);
    }
    else
    {
        lastError = NO_NEW_DATA;
        return lastSample;
    }
}

VN300Data VN300::sampleASCII()
{
    usart.clearQueue();
    // Returns Quaternion, Magnetometer, Accelerometer and Gyro
    usart.writeString(preSampleImuString);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        lastError = BUS_FAULT;
        return lastSample;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_WARN(logger, "VN300 sampling message invalid checksum");
        // If something goes wrong i return the last sampled data
        lastError = BUS_FAULT;
        return lastSample;
    }

    QuaternionData quat   = sampleQuaternion();
    MagnetometerData mag  = sampleMagnetometer();
    AccelerometerData acc = sampleAccelerometer();
    GyroscopeData gyro    = sampleGyroscope();

    usart.clearQueue();
    // Returns INS LLA message
    usart.writeString(preSampleINSlla);

    if (!recvStringCommand(recvString.data(), recvStringMaxDimension))
    {
        LOG_WARN(logger, "Unable to sample due to serial communication error");
        lastError = BUS_FAULT;
        return lastSample;
    }

    if (!verifyChecksum(recvString.data(), recvStringLength))
    {
        LOG_WARN(logger, "VN300 sampling message invalid checksum");
        lastError = BUS_FAULT;
        return lastSample;
    }

    VN300Defs::Ins_Lla ins = sampleIns();

    return VN300Data(quat, mag, acc, gyro, ins);
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
    uint16_t outputGroup =
        VN300Defs::BINARYGROUP_COMMON | VN300Defs::BINARYGROUP_GPS;

    uint16_t commonGroup =
        VN300Defs::COMMONGROUP_YAWPITCHROLL |
        VN300Defs::COMMONGROUP_QUATERNION | VN300Defs::COMMONGROUP_ANGULARRATE |
        VN300Defs::COMMONGROUP_VELOCITY | VN300Defs::COMMONGROUP_ACCEL |
        VN300Defs::COMMONGROUP_MAGPRES | VN300Defs::COMMONGROUP_INSSTATUS;

    uint16_t gnssGroup = VN300Defs::GPSGROUP_NUMSATS | VN300Defs::GPSGROUP_FIX |
                         VN300Defs::GPSGROUP_POSLLA;

    // The "comp" string it's created in order format the message and also
    // compare the reply. The fields represents the binary register "75", the
    // type of communication "0" means that no asynchronous message is active
    // and "8" represents the divider at which the asynchronous message is sent
    // respect to the max rate
    const char *comp = "VNWRG,75,0,8";

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

VN300Defs::Ins_Lla VN300::sampleIns()
{
    unsigned int indexStart = 0;
    char *nextNumber;
    VN300Defs::Ins_Lla data;

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

    // Add the timestamp to the parsed data
    data.insTimestamp = TimestampTimer::getTimestamp();
    // Parse the data skipping time and week of the gps
    strtod(recvString.data() + indexStart, &nextNumber);
    strtol(nextNumber + 1, &nextNumber, 16);
    data.status =
        static_cast<uint16_t>(strtol(nextNumber + 1, &nextNumber, 16));
    data.fix_ins   = data.status & 0x03;
    data.fix_gps   = (data.status >> 2) & 0x01;
    data.yaw       = strtof(nextNumber + 1, &nextNumber);
    data.pitch     = strtof(nextNumber + 1, &nextNumber);
    data.roll      = strtof(nextNumber + 1, &nextNumber);
    data.latitude  = strtod(nextNumber + 1, &nextNumber);
    data.longitude = strtod(nextNumber + 1, &nextNumber);
    data.altitude  = strtod(nextNumber + 1, &nextNumber);
    data.nedVelX   = strtof(nextNumber + 1, &nextNumber);
    data.nedVelY   = strtof(nextNumber + 1, &nextNumber);
    data.nedVelZ   = strtof(nextNumber + 1, NULL);

    return data;
}

}  // namespace Boardcore
