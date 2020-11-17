/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rightd
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "math/Vec3.h"

/**
 * @brief Define all the possible sensor identifiers.
 */
enum SensorID : uint8_t
{
    BASE,
    BAROMETER,
    TEMPERATURE,
    ACCELEROMETER,
    GYROSCOPE,
    MAGNETOMETER,
    IMU_9DOF,
    GPS,
    LAST_SENSOR_ID  // number of SensorID members
};

/**
 * @brief Base sensor data structure, containing only the
 *        sensor ID and the timestamp.
 *        Notice that both the base class and the derived ones must be 
 *        trivially copiable in order for them to be correctly logged.
 */
struct SensorData
{
    unsigned long long timestamp;
    SensorID id;

    SensorData(SensorID id = SensorID::BASE) : id(id) {}
};

/**
 * @brief Structure to handle temperature data.
 */
struct TemperatureData : public SensorData
{
    float temperature;

    TemperatureData() : SensorData(SensorID::TEMPERATURE) {}
};

/**
 * @brief Structure to handle pressure data.
 */
struct BarometerData : public SensorData
{
    float pressure;

    BarometerData() : SensorData(SensorID::BAROMETER) {}
};

/**
 * @brief Structure to handle accelerometer, gyroscope and magnetometer data.
 *        Type can be either accelerometer, gyroscope or magnetometer.
 */
struct MotionSensorData : public SensorData
{
    Vec3 vector;

    MotionSensorData(SensorID id = SensorID::ACCELEROMETER) : SensorData(id) {} 
};

/**
 * @brief Structure to handle accelerometer data.
 */
struct AccelerometerData : public MotionSensorData
{
    AccelerometerData() : MotionSensorData(SensorID::ACCELEROMETER) {}
};

/**
 * @brief Structure to handle gyroscope data.
 */
struct GyroscopeData : public MotionSensorData
{
    GyroscopeData() : MotionSensorData(SensorID::GYROSCOPE) {}
};

/**
 * @brief Structure to handle magnetometer data.
 */
struct MagnetometerData : public MotionSensorData
{
    MagnetometerData() : MotionSensorData(SensorID::MAGNETOMETER) {}
};

/**
 * @brief Structure to handle 9 dof IMU data.
 */
struct IMU9DofData : public SensorData
{
    AccelerometerData accel;
    GyroscopeData gyro;
    MagnetometerData magneto;

    IMU9DofData() : SensorData(SensorID::IMU_9DOF) {}
};

/**
 * @brief Structure to handle GPS data.
 */
struct GPSData : public SensorData
{
    float latitude;          /**< [deg] */
    float longitude;         /**< [deg] */
    float height;            /**< [m]   */
    float velocity_north;    /**< [m/s] */
    float velocity_east;     /**< [m/s] */
    float velocity_down;     /**< [m/s] */
    float speed;             /**< [m/s] */
    uint8_t num_satellites;  /**< [1]   */
    bool fix;

    GPSData() : SensorData(SensorID::GPS) {}
};