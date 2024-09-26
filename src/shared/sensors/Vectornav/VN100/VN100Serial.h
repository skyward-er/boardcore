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
 * The VN100S sensor is a calibrated IMU which includes accelerometer,
 * magnetometer, gyroscope, barometer and temperature sensor. The device
 * provides also a calibration matrix and an anti-drift matrix for the gyroscope
 * values. The goal of this driver though is to interface the sensor in its
 * basic use. Things like asynchronous data and anti-drift techniques haven't
 * been implemented yet. The driver is intended to be used with the "Rugged
 * sensor" version (aka only UART communication) although the actual VN100S chip
 * is capable also of SPI communication.
 *
 * The VN100S supports both binary and ASCII encoding for communication but via
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
