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

#pragma once

/**
 * @brief Driver for the VN300S IMU.
 *
 * The VN300S sensor is a calibrated IMU which includes accelerometer,
 * magnetometer, gyroscope, barometer and temperature sensor. The device
 * provides also a calibration matrix and an anti-drift matrix for the gyroscope
 * values. The goal of this driver though is to interface the sensor in its
 * basic use. Things like asynchronous data and anti-drift techniques haven't
 * been implemented yet. The driver is intended to be used with the "Rugged
 * sensor" version (aka only UART communication) although the actual VN300S chip
 * is capable also of SPI communication.
 *
 * The VN300S supports both binary and ASCII encoding for communication but via
 * serial and with the asynchronous mode disabled only ASCII is available. The
 * protocol also provides two algorithms to verify the integrity of the messages
 * (8 bit checksum and 16 bit CRC-CCITT) both selectable by the user using the
 * constructor method. The serial communication also can be established with
 * various baud rates:
 * - 9600
 * - 19200
 * - 38400
 * - 57600
 * - 115200
 * - 128000
 * - 230400
 * - 460800
 * - 921600
 */

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <fmt/format.h>
#include <sensors/Sensor.h>
#include <sensors/Vectornav/VNCommonSerial.h>
#include <string.h>
#include <utils/Debug.h>

#include <Eigen/Core>

#include "VN300Data.h"
#include "VN300Defs.h"
#include "drivers/usart/USART.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN300 IMU.
 */
class VN300 : public Sensor<VN300Data>, public VNCommonSerial
{
public:
    /**
     * @brief Constructor.
     *
     * @param usart Serial bus used for the sensor.
     * @param BaudRate different from the sensor's default [9600, 19200, 38400,
     * 57600, 115200, 128000, 230400, 460800, 921600].
     * @param Redundancy check option.
     * @param samplePeriod Sampling period in ms
     * @param antPos antenna A position
     */
    VN300(USART& usart, int userBaudRate,
          VN300Defs::SamplingMethod samplingMethod =
              VN300Defs::SamplingMethod::BINARY,
          CRCOptions crc                     = CRCOptions::CRC_ENABLE_8,
          VN300Defs::AntennaPosition antPosA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          VN300Defs::AntennaPosition antPosB = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          Eigen::Matrix3f rotMat             = Eigen::Matrix3f::Identity());

    bool init() override;

    /**
     * @brief Method to reset the sensor to default values and to close
     * the connection. Used if you need to close and re initialize the sensor.
     *
     * @return True if operation succeeded.
     */
    bool closeAndReset();
    // TODO: move to common files

    bool selfTest() override;

private:
    /**
     * @brief Sample action implementation.
     */
    VN300Data sampleImpl() override;

    /**
     * @brief Method to find the baudrate of the sensor at startup
     *
     * @return True if operation succeeded.
     */
    bool findBaudrate();
    // TODO: should we keep it? maybe in common files?

    /**
     * @brief Disables the async messages that the VN300 is default configured
     * to send at 40Hz on startup.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool disableAsyncMessages(bool waitResponse = true);
    // TODO: move to common files

    /**
     * @brief Configures the default serial communication.
     */
    void configDefaultSerialPort();
    // TODO: move to common files

    /**
     * @brief Configures the user defined serial communication.
     *
     * @return True if operation succeeded.
     */
    bool configUserSerialPort();
    // TODO: move to common files

    /**
     * @brief Sets the user selected crc method.
     *
     * @param waitResponse If true wait for a serial response.
     *
     * @return True if operation succeeded.
     */
    bool setCrc(bool waitResponse = true);
    // TODO: move to common files

    /**
     * @brief Write the settings on the non volatile-memory.
     *
     * @return True if operation succeeded.
     */
    bool writeSettingsCommand();
    // TODO: is it used? maybe can be placed in common files?

    /**
     * @brief Sets the antenna A offset.
     *
     * @param antPos antenna position.
     *
     * @return True if operation succeeded.
     */
    bool setAntennaA(VN300Defs::AntennaPosition antPos);

    /**
     * @brief Sets the compass baseline, position offset of antenna B respect to
     * antenna A. Uncertainty must be higher than actual measurement error,
     * possibly twice as the error.
     * All measures are in meters [m].
     *
     * @param antPos antenna position.
     *
     * @return True if operation succeeded.
     */
    bool setCompassBaseline(VN300Defs::AntennaPosition antPos);

    /**
     * @brief Set the reference frame rotation of the sensor in order to have
     * all the data on the desired reference frame.
     *
     * @param rotMat rotation matrix.
     *
     * @return True if operation succeeded.
     */
    bool setReferenceFrame(Eigen::Matrix3f rotMat);

    /**
     * @brief Set the binary output register
     *
     * @return True if operation succeeded.
     */
    bool setBinaryOutput();
    // TODO: refactor

    /**
     * @brief Method implementation of self test.
     *
     * @return True if operation succeeded.
     */
    bool selfTestImpl();

    VN300Defs::Ins_Lla sampleIns();

    VN300Data sampleBinary();

    VN300Data sampleASCII();

    /**
     * @brief Sends the command to the sensor with the correct checksum added
     * so '*' symbol is not needed at the end of the string as well as the '$'
     * at the beginning of the command.
     *
     * @param command Command to send.
     *
     * @return True if operation succeeded.
     */
    bool sendStringCommand(std::string command);
    // TODO: put in common files

    /**
     * @brief Receives a command from the VN300 serialInterface->recv() but
     * swaps the first \n with a \0 to close the message.
     *
     * @param command The char array which will be filled with the command.
     * @param maxLength Maximum length for the command array.
     *
     * @return True if operation succeeded.
     */
    bool recvStringCommand(char* command, int maxLength);
    // TODO: put in common files

    /**
     * @brief Receives binary data and parse directly into BinaryData struct
     * which has the __attribute__(packed)
     *
     * @param bindata passed as reference
     *
     * @return true if operation succeeded
     *
     */
    bool sampleBin(VN300Defs::BinaryData& bindata);
    // TODO: can be removed and placed inside sampleBinary()

    /**
     * @brief Default baudrate value for the usart communication.
     */
    static const int defaultBaudRate = 115200;

    VN300Defs::SamplingMethod samplingMethod;
    bool isInit = false;

    VN300Defs::AntennaPosition antPosA;
    VN300Defs::AntennaPosition antPosB;
    Eigen::Matrix3f rotMat;

    /**
     * @brief IMU pre-elaborated sample string for efficiency reasons.
     */
    const char* preSampleImuString = "";

    /**
     * @brief Temperature and pressure pre-elaborated sample string for
     * efficiency reasons.
     */
    const char* preSampleINSlla = "";

    /**
     * @brief Pre-elaborated binary output polling command.
     */
    const char* preSampleBin1 = "";
};
}  // namespace Boardcore
