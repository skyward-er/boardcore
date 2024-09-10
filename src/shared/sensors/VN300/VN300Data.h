/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Lorenzo Cucchi
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

#include <sensors/SensorData.h>

namespace Boardcore
{

/**
 * @brief Structure to handle antenna A position units [m]
 */
struct AntennaPosition  // TODO: move inside defs
{
    float posX;
    float posY;
    float posZ;
    float uncX;
    float uncY;
    float uncZ;
};

/**
 * @brief Structure to handle INS data
 */
struct Ins_Lla  // TODO: move inside defs, should this go inside SensorData.h?
{
    uint64_t insTimestamp;
    uint16_t fix_gps;
    uint16_t fix_ins;
    uint16_t status;
    float yaw;
    float pitch;
    float roll;
    float latitude;
    float longitude;
    float altitude;
    float nedVelX;
    float nedVelY;
    float nedVelZ;
};

/**
 * @brief Structure to handle binary message
 *
 * @property The struct needs the packed attribute in order to have contiguous
 * memory allocation in order to be able to parse directly the received message
 */
struct __attribute__((packed)) BinaryData  // TODO: move inside defs
{
    uint8_t group;
    uint16_t common;
    uint16_t gnss;
    float yaw_bin;
    float pitch_bin;
    float roll_bin;
    float quatX_bin;
    float quatY_bin;
    float quatZ_bin;
    float quatW_bin;
    float angx;
    float angy;
    float angz;
    float velx;
    float vely;
    float velz;
    float accx;
    float accy;
    float accz;
    float magx;
    float magy;
    float magz;
    float temp;
    float pres;
    uint16_t ins_status;
    uint8_t numsats;
    uint8_t fix;
    double latitude_bin;
    double longitude_bin;
    double altitude_bin;
    uint16_t checksum;
};

/**
 * @brief data type class
 */
struct VN300Data : public QuaternionData,
                   public MagnetometerData,
                   public AccelerometerData,
                   public GyroscopeData,
                   public Ins_Lla
{

    /**
     * @brief Void parameters constructor
     */
    // cppcheck-suppress uninitDerivedMemberVar
    VN300Data()
        : QuaternionData{0, 0.0, 0.0, 0.0, 0.0}, MagnetometerData{0, 0.0, 0.0,
                                                                  0.0},
          AccelerometerData{0, 0.0, 0.0, 0.0}, GyroscopeData{0, 0.0, 0.0, 0.0},
          Ins_Lla{0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    {
    }

    /**
     * @brief Constructor with parameters
     *
     * @param single data structures for all the data
     */
    // cppcheck-suppress uninitDerivedMemberVar
    VN300Data(const QuaternionData& quat, const MagnetometerData& magData,
              const AccelerometerData& accData, const GyroscopeData& gyro,
              const Ins_Lla& ins)
        : QuaternionData(quat), MagnetometerData(magData),
          AccelerometerData(accData), GyroscopeData(gyro), Ins_Lla(ins)
    {
    }

    static std::string header()
    {
        return "quaternionTimestamp,quaternionX,quaternionY,quaternionZ,"
               "quaternionW,magneticFieldTimestamp,"
               "magneticFieldX,magneticFieldY,magneticFieldZ,"
               "accelerationTimestamp,accelerationX,accelerationY,"
               "accelerationZ,angularSpeedTimestamp,angularSpeedX,"
               "angularSpeedY,angularSpeedZ,insTimeStamp,"
               "status,yaw,pitch,roll,latitude,"
               "longitude,altitude,nedVelX,nedVelY,nedVelZ\n";
    }

    void print(std::ostream& os) const
    {
        os << quaternionTimestamp << "," << quaternionX << "," << quaternionY
           << "," << quaternionZ << "," << quaternionW << ","
           << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << ","
           << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << angularSpeedTimestamp << "," << angularSpeedX << ","
           << angularSpeedY << "," << angularSpeedZ << "," << insTimestamp
           << "," << status << "," << yaw << "," << pitch << "," << roll << ","
           << latitude << "," << longitude << "," << altitude << "," << nedVelX
           << "," << nedVelY << "," << nedVelZ << "\n";
    }
};

}  // namespace Boardcore
