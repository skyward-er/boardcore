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
#pragma once

#include <string>
#include <fmt/format.h>

#include <sensors/Sensor.h>
#include "VN100Data.h"
#include "VN100Serial.h"

/**
 * @brief Header class for the VN100 IMU
 */
class VN100 : public Sensor<VN100Data>
{

private:
	
    /**
     * @brief default vn100 baud rate
     */
    static const unsigned int defaultBaudRate    = 115200;

    /** 
     * @brief default USART port number
     */
    static const unsigned int defaultPortNumber  = 2;

    /**
     * @brief baud rate defined by the user
     */
    unsigned int baudRate;

    /**
     * @brief USART port number
     */
    unsigned int portNumber;

    /**
     * @brief redundancy check option
     */
    uint8_t crc;

    /**
     * @brief initialization state
     */
    bool isInit;

    /**
     * @brief serial interface that is needed to communicate
     * with the sensor via ASCII codes
     */
    VN100Serial *serialInterface;

    /**
     * @brief method to configure the serial communication
     */
    bool configSerialPort();

    /**
     * @brief sends the command to the sensor with the correct checksum added
     * so '*' symbol is needed at the end of the string
     * 
     * @return boolean that confirms or not the success
     */
    bool sendStringCommand(std::string command);

    /**
     * @brief method to calculate 8bit vector checksum 8bit
     */
    uint8_t calculateChecksum8(uint8_t * message, int length);

    /**
     * @brief method to calculate a 8bit vector checksum 16bit
     */
    uint16_t calculateChecksum16(uint8_t * message, int length);

    /**
     * @brief sample action implementation
     */
    VN100Data sampleImpl() override;

public:

    /**
     * @brief configuration constants for redundancy check
     */
    static const uint8_t CRC_NO         = 0x00;
    static const uint8_t CRC_ENABLE_8   = 0x08;
    static const uint8_t CRC_ENABLE_16  = 0x10;

    /**
     * @brief Constructor
     */
    VN100();

    /**
     * @brief Constructor
     * @param USART port number
     * @param baudRate different from the sensor's default
     */
	VN100(unsigned int portNumber, unsigned int baudRate);

    /**
     * @brief Constructor
     * @param USART port number
     * @param baudRate different from the sensor's default
     * @param redundancy check option
     */
    VN100(unsigned int portNumber, unsigned int baudRate, uint8_t crc);

    /**
     * @brief Init method to initialize the IMU and to set 
     * the user defined working conditions
     * @return Boolean value indicating the operation success state
     */
    bool init() override;

    /**
     * @brief Method to implement the verification process to ensure
     * that the sensor is up and running
     * @return Boolean of the result
     */
    bool selfTest() override;
};