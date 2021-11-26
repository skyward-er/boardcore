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

#include <sensors/SensorData.h>

/**
 * @brief Structure to handle quaternion data
 */
struct QuaternionData
{
    uint64_t quat_timestamp;
    float quat_x;
    float quat_y;
    float quat_z;
    float quat_w;
};

/**
 * @brief data type class
 */
struct VN100Data : public QuaternionData,
                   public MagnetometerData, 
                   public AccelerometerData, 
                   public GyroscopeData, 
                   public TemperatureData, 
                   public PressureData
{

    /**
     * @brief Void parameters constructor
     */
    VN100Data() : QuaternionData    {0, 0.0, 0.0, 0.0, 0.0},
                  MagnetometerData  {0, 0.0, 0.0, 0.0},
                  AccelerometerData {0, 0.0, 0.0, 0.0},
                  GyroscopeData     {0, 0.0, 0.0, 0.0},
                  TemperatureData   {0, 0.0},
                  PressureData      {0, 0.0}{}

    /**
     * @brief Constructor with parameters
     * 
     * @param single data structures for all the data
     */ 
    VN100Data(QuaternionData quat, MagnetometerData magData, AccelerometerData accData, GyroscopeData gyro, TemperatureData temp, PressureData pres)
                : QuaternionData    {quat.quat_timestamp,       quat.quat_x,        quat.quat_y,        quat.quat_z, quat.quat_w},
                  MagnetometerData  {magData.mag_timestamp,     magData.mag_x,      magData.mag_y,      magData.mag_z},
                  AccelerometerData {accData.accel_timestamp,   accData.accel_x,    accData.accel_y,    accData.accel_z},
                  GyroscopeData     {gyro.gyro_timestamp,       gyro.gyro_x,        gyro.gyro_y,        gyro.gyro_z},
                  TemperatureData   {temp.temp_timestamp,       temp.temp},
                  PressureData      {pres.press_timestamp,      pres.press}{}


    static std::string header()
    {
        return  "quat_timestamp,quat_x,quat_y,quat_z,quat_w,mag_timestamp,mag_x,mag_y,mag_z,"
                "accel_timestamp,accel_x,accel_y,accel_z,gyro_timestamp,gyro_x,gyro_y,gyro_z"
                "temp_timestamp,temp,press_timestamp,press\n";
    }

    void print(std::ostream& os) const
    {
        os  << quat_timestamp   << "," << quat_x    << "," << quat_y    << "," << quat_z    << "," << quat_w << ","
            << mag_timestamp    << "," << mag_x     << "," << mag_y     << "," << mag_z     << ","
            << accel_timestamp  << "," << accel_x   << "," << accel_y   << "," << accel_z   << ","
            << gyro_timestamp   << "," << gyro_x    << "," << gyro_y    << "," << gyro_z    << ","
            << temp_timestamp   << "," << temp      << ","
            << press_timestamp  << "," << press     << "\n";
    }
};