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

#pragma once

/**
 * @brief Driver for the VN100S IMU.
 *
 * The VN100 sensor is a calibrated IMU which includes accelerometer,
 * magnetometer, gyroscope, barometer and temperature sensor. It also provides
 * attitude data (yaw, pith, roll, quaternion). The device provides also a
 * calibration matrix and an anti-drift matrix for the gyroscope values.
 *
 * This driver samples IMU compensated data (accelerometer, gyroscope and
 * magnetometer), quaternion data, temperature and pressure data.
 * The sampling rate is 400Hz.
 *
 * This driver only supports binary encoding for communication.
 */

#include <sensors/Sensor.h>
#include <sensors/Vectornav/VNCommonSerial.h>

#include "VN100SerialData.h"

namespace Boardcore
{

/**
 * @brief Driver class for VN100 IMU.
 */
class VN100Serial : public Sensor<VN100SerialData>, public VNCommonSerial
{
public:
    /**
     * @brief Constructor.
     *
     * @param usart Serial bus used for the sensor.
     * @param BaudRate different from the sensor's default [9600, 19200, 38400,
     * 57600, 115200, 128000, 230400, 460800, 921600].
     * @param crc Checksum option.
     * @param timeout The maximum time that will be waited when reading from the
     * sensor.
     */
    VN100Serial(USART& usart, int baudrate, CRCOptions crc,
                std::chrono::milliseconds timeout);

    bool init() override;

    bool selfTest() override;

protected:
    /**
     * @brief Sample action implementation.
     */
    VN100SerialData sampleImpl() override;

private:
    /**
     * @brief Struct used to store the binary data received from the sensor.
     */
    struct __attribute__((packed)) BinaryData
    {
        uint8_t group;
        uint16_t group1;
        float quaternionX;
        float quaternionY;
        float quaternionZ;
        float quaternionW;
        float angularX;
        float angularY;
        float angularZ;
        float accelerationX;
        float accelerationY;
        float accelerationZ;
        float magneticFieldX;
        float magneticFieldY;
        float magneticFieldZ;
        float temperature;
        float pressure;
        uint16_t crc;
    };

    /**
     * @brief Build output data packet starting from raw binary data received
     * from the sensor.
     *
     * @param rawData The raw data received from the sensor.
     * @param data The structure that will contain the output.
     * @param timestamp The timestamp of the extracted data.
     */
    void buildBinaryData(const BinaryData& binData, VN100SerialData& data,
                         const uint64_t sampleTimestamp);

    /**
     * @brief Set the binary output register.
     *
     * @return True if operation succeeded.
     */
    bool setBinaryOutput();

    /**
     * @brief Pre computed command used to ask a binary sample to the sensor.
     */
    const char* askSampleCommand = "";
};
}  // namespace Boardcore
