/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Lorenzo Cucchi, Fabrizio Monti
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

#include <sensors/Sensor.h>
#include <sensors/Vectornav/VNCommonSerial.h>

#include "VN300Data.h"
#include "VN300Defs.h"

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
     * @param sampleOption The data packet we want to sample.
     * @param crc Checksum option.
     * @param timeout The maximum time that will be waited when reading from the
     * sensor.
     * @param antPosA Antenna position A.
     * @param antPosB Antenna position B.
     * @param rotMat Rotation matrix.
     */
    VN300(USART& usart, int userBaudRate, VN300Defs::SampleOptions sampleOption,
          CRCOptions crc, std::chrono::milliseconds timeout,
          VN300Defs::AntennaPosition antPosA = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          VN300Defs::AntennaPosition antPosB = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0},
          Eigen::Matrix3f rotMat             = Eigen::Matrix3f::Identity());

    bool init() override;

    bool selfTest() override;

protected:
    /**
     * @brief Sample action implementation.
     */
    VN300Data sampleImpl() override;

private:
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
     * @brief Set the binary output register.
     *
     * @return True if operation succeeded.
     */
    bool setBinaryOutput();

    /**
     * @brief Utility function for sampling data from the sensor (FULL packet).
     *
     * @return The sampled data.
     */
    VN300Data sampleFull();

    /**
     * @brief Utility function for sampling data from the sensor (Reduced
     * packet).
     *
     * @return The sampled data.
     */
    VN300Data sampleReduced();

    /**
     * @brief Build output data packet starting from raw binary data received
     * from the sensor.
     *
     * @param rawData The raw data received from the sensor.
     * @param data The structure that will contain the output.
     * @param timestamp The timestamp of the extracted data.
     */
    void buildBinaryDataFull(const VN300Defs::BinaryDataFull& rawData,
                             VN300Data& data, const uint64_t timestamp);

    /**
     * @brief Build output data packet starting from raw binary data received
     * from the sensor.
     *
     * @param rawData The raw data received from the sensor.
     * @param data The structure that will contain the output.
     * @param timestamp The timestamp of the extracted data.
     */
    void buildBinaryDataReduced(const VN300Defs::BinaryDataReduced& rawData,
                                VN300Data& data, const uint64_t timestamp);

    VN300Defs::SampleOptions sampleOption;

    VN300Defs::AntennaPosition antPosA;
    VN300Defs::AntennaPosition antPosB;
    Eigen::Matrix3f rotMat;

    /**
     * @brief Pre-elaborated binary output polling command.
     */
    const char* preSampleBin1 = "";
};

}  // namespace Boardcore
