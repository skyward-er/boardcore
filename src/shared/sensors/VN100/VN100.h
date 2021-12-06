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

/** BRIEF INTRODUCTION
 * The VN100S sensor is a calibrated IMU which has inside various sensors
 * such as Accelerometer, Magnetometer, Gyroscope, Barometer and Temperature Sensor.
 * This sensor provides also a calibration matrix and an anti-drift matrix for the 
 * gyroscope values. The goal of this driver though is to interface the sensor
 * in its basic use. Things like asynchronous data and anti-drift techniques haven't
 * been implemented yet. 
 * The driver is intended to be used with the "Rugged sensor" version (aka only UART communication)
 * although the actual VN100S chip is capable also of SPI communication.
 * 
 * COMMUNICATION PROTOCOL
 * The VN100S supports both binary and ASCII encoding for communication but via serial 
 * and with the asynchronous mode disabled only ASCII is used.
 * The protocol also provides two algorithms to very the integrity of the messages
 * (8 bit checksum and 16 bit CRC-CCITT) both selectable by the user using the constructor
 * method. The serial communication also can be established with various baud rates:
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
#pragma once

#include <Debug.h>
#include <string.h>
#include <fmt/format.h>
#include <sensors/Sensor.h>
#include <diagnostic/PrintLogger.h>

#include "VN100Data.h"
#include "VN100Serial.h"

namespace Boardcore
{
    /**
     * @brief Header class for the VN100 IMU
     */
    class VN100 : public Sensor<VN100Data>
    {

    private:

        /** 
         * @brief Default USART port number
         */
        static const unsigned int defaultPortNumber         = 2;

        /**
         * @brief Max received string lengh
         */
        static const unsigned int recvStringMaxDimension    = 200;

        /**
         * @brief Baud rate defined by the user
         */
        unsigned int baudRate;

        /**
         * @brief USART port number
         */
        unsigned int portNumber;

        /**
         * @brief Redundancy check option
         */
        uint8_t crc;

        /**
         * @brief Initialization state
         */
        bool isInit;

        /**
         * @brief IMU pre-elaborated sample string for efficiency reasons
         */
        string *preSampleImuString;

        /**
         * @brief Temperature and Pressure pre-elaborated sample string for efficiency reasons
         */
        string *preSampleTempPressString;

        /**
         * @brief Pointer to the received string by the sensor
         * Allocated 1 time only (200 bytes)
         */
        char * recvString;

        /**
         * @brief Actual strlen() of the recvString
         */
        unsigned int recvStringLength;

        /**
         * @brief Serial interface that is needed to communicate
         * with the sensor via ASCII codes
         */
        VN100Serial *serialInterface;

        /**
         * @brief Logger
         */
        PrintLogger logger = Logging::getLogger("vn100");

        /**
         * @brief This method disables the async messages that the vn100
         * is default configured to send at 40Hz on startup
         * 
         * @param Boolean that asks if it should wait for a serial response
         * 
         * @return Boolean that confirms or not the success
         */ 
        bool disableAsyncMessages(bool waitResponse);

        /**
         * @brief Method to configure the default serial communication
         * 
         * @return Boolean that confirms or not the success
         */
        bool configDefaultSerialPort();

        /**
         * @brief Method to configure the user defined serial
         * communication
         * 
         * @return Boolean that confirms or not the success
         */
        bool configUserSerialPort();

        /**
         * @brief Method to set the user selected crc method
         * 
         * @param Boolean that asks if it should wait for a serial response
         * 
         * @return Boolean that confirms or not the success
         */
        bool setCrc(bool waitResponse);

        /**
         * @brief Method implementation of self test
         * 
         * @return Boolean that confirms or not the self test success
         */
        bool selfTestImpl();

        /**
         * @brief Sample action implementation
         */
        VN100Data sampleImpl() override;

        /**
         * @brief Sample only the quaternion
         * 
         * @return Quaternion data declared in VN100Data
         */
        QuaternionData sampleQuaternion();

        /**
         * @brief Sample only the magnetometer
         * 
         * @return Magnetometer data declared in SensorData
         */
        MagnetometerData sampleMagnetometer();

        /**
         * @brief Sample only the accelerometer
         * 
         * @return Accelerometer data declared in SensorData
         */
        AccelerometerData sampleAccelerometer();

        /**
         * @brief Sample only the gyroscope
         * 
         * @return Gyroscope data declared in SensorData
         */
        GyroscopeData sampleGyroscope();

        /**
         * @brief Sample only the temperature
         * 
         * @return Temperature data declared in SensorData
         */
        TemperatureData sampleTemperature();

        /**
         * @brief Sample only the pressure
         * 
         * @return Pressure data declared in SensorData
         */
        PressureData samplePressure();

        /**
         * @brief Sends the command to the sensor with the correct checksum added
         * so '*' symbol is not needed at the end of the string as well as the '$'
         * at the beginning of the command
         * 
         * @return Boolean that confirms or not the success
         */
        bool sendStringCommand(std::string command);

        /**
         * @brief Receives the vn100 command with serialInterface -> recv() but
         * swaps the first \n with a \0 to close the message
         * 
         * @param The char vector which will be filled with the command
         * 
         * @return Boolean that confirms or not the success
         */
        bool recvStringCommand(char * command, int maxLength);

        /**
         * @brief Method to verify the crc validity of a command
         * 
         * @param The char vector which contains the command
         * @param Its length
         * 
         * @return Boolean that confirms or not the command validity
         */
        bool verifyChecksum(char * command, int maxLength);

        /**
         * @brief Method to calculate 8bit vector checksum 8bit
         * 
         * @return The 8 bit checksum
         */
        uint8_t calculateChecksum8(uint8_t * message, int length);

        /**
         * @brief Method to calculate a 8bit vector checksum 16bit
         * 
         * @return The 16 bit CRC16-CCITT error check
         */
        uint16_t calculateChecksum16(uint8_t * message, int length);

    public:

        /**
         * @brief Configuration enumeration for redundancy check
         */
        enum CRCOptions : uint8_t
        {
            CRC_NO          = 0x00,
            CRC_ENABLE_8    = 0x08,
            CRC_ENABLE_16   = 0x10
        };

        /**
         * @brief Configuration enumeration for baud rate
         */
        enum BaudRates : unsigned int
        {
            Baud_9600       = 9600,
            Baud_19200      = 19200,
            Baud_38400      = 38400,
            Baud_57600      = 57600,
            Baud_115200     = 115200,
            Baud_128000     = 128000,
            Baud_230400     = 230400,
            Baud_460800     = 460800,
            Baud_921600     = 921600
        };

        /**
         * @brief Constructor
         */
        VN100();

        /**
         * @brief Constructor
         * 
         * @param USART port number
         * @param BaudRate different from the sensor's default
         */
	    VN100(unsigned int portNumber, unsigned int baudRate);

        /**
         * @brief Constructor
         * 
         * @param USART port number
         * @param BaudRate different from the sensor's default
         * @param Redundancy check option
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
}
