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

#include <Debug.h>
#include <string.h>
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
    static const unsigned int defaultBaudRate           = 115200;

    /** 
     * @brief default USART port number
     */
    static const unsigned int defaultPortNumber         = 2;

    /**
     * @brief max received string lengh
     */
    static const unsigned int recvStringMaxDimension    = 200;

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
     * @brief pointer to the received string by the sensor
     * Allocated 1 time only (200 bytes)
     */
    char * recvString;

    /**
     * @brief actual strlen() of the recvString
     */
    unsigned int recvStringLength;

    /**
     * @brief serial interface that is needed to communicate
     * with the sensor via ASCII codes
     */
    VN100Serial *serialInterface;

    /**
     * @brief this method disables the async messages that the vn100
     * is default configured to send at 40Hz on startup
     * 
     * @return boolean that confirms or not the success
     */ 
    bool disableAsyncMessages();

    /**
     * @brief method to configure the default serial communication
     * 
     * @return boolean that confirms or not the success
     */
    bool configDefaultSerialPort();

    /**
     * @brief method to configure the user defined serial
     * communication
     * 
     * @return boolean that confirms or not the success
     */
    bool configUserSerialPort();

    /**
     * @brief method to set the user selected crc method
     * 
     * @return boolean that confirms or not the success
     */
    bool setCrc();

    /**
     * @brief method implementation of self test
     * 
     * @return boolean that confirms or not the self test success
     */
    bool selfTestImpl();

    /**
     * @brief sample action implementation
     */
    VN100Data sampleImpl() override;

    /**
     * @brief sample only the quaternion
     * 
     * @return Quaternion data declared in VN100Data
     */
    QuaternionData sampleQuaternion();

    /**
     * @brief sample only the magnetometer
     * 
     * @return Magnetometer data declared in SensorData
     */
    MagnetometerData sampleMagnetometer();

    /**
     * @brief sample only the accelerometer
     * 
     * @return Accelerometer data declared in SensorData
     */
    AccelerometerData sampleAccelerometer();

    /**
     * @brief sample only the gyroscope
     * 
     * @return Gyroscope data declared in SensorData
     */
    GyroscopeData sampleGyroscope();

    /**
     * @brief sample only the temperature
     * 
     * @return Temperature data declared in SensorData
     */
    TemperatureData sampleTemperature();

    /**
     * @brief sample only the pressure
     * 
     * @return Pressure data declared in SensorData
     */
    PressureData samplePressure();

    /**
     * @brief sends the command to the sensor with the correct checksum added
     * so '*' symbol is not needed at the end of the string as well as the '$'
     * at the beginning of the command
     * 
     * @return boolean that confirms or not the success
     */
    bool sendStringCommand(std::string command);

    /**
     * @brief receives the vn100 command with serialInterface -> recv() but
     * swaps the first \n with a \0 to close the message
     * 
     * @param the char vector which will be filled with the command
     * 
     * @return boolean that confirms or not the success
     */
    bool recvStringCommand(char * command, int maxLength);

    /**
     * @brief method to verify the crc validity of a command
     * 
     * @param the char vector which contains the command
     * @param its length
     * 
     * @return boolean that confirms or not the command validity
     */
    bool verifyChecksum(char * command, int maxLength);

    /**
     * @brief method to calculate 8bit vector checksum 8bit
     * 
     * @return the 8 bit checksum
     */
    uint8_t calculateChecksum8(uint8_t * message, int length);

    /**
     * @brief method to calculate a 8bit vector checksum 16bit
     * 
     * @return the 16 bit CRC16-CCITT error check
     */
    uint16_t calculateChecksum16(uint8_t * message, int length);

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
     * 
     * @param USART port number
     * @param baudRate different from the sensor's default
     */
	VN100(unsigned int portNumber, unsigned int baudRate);

    /**
     * @brief Constructor
     * 
     * @param USART port number
     * @param baudRate different from the sensor's default
     * @param redundancy check option
     */
    VN100(unsigned int portNumber, unsigned int baudRate, uint8_t crc);

    /**
     * @brief Init method to initialize the IMU and to set 
     * the user defined working conditions
     * 
     * @return Boolean value indicating the operation success state
     */
    bool init() override;

    /**
     * @brief Method to implement the verification process to ensure
     * that the sensor is up and running
     * 
     * @return Boolean of the result
     */
    bool selfTest() override;

    /**
     * @brief Method to reset the sensor to default values and to close
     * the connection. Used if you need to close and re initialize the sensor
     * 
     * @return Boolean of the result
     */
    bool closeAndReset();
};