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

#include <cstdint>

/**
 * @brief Structure to handle sensor data timestamp.
 */
struct TimestampData
{
    uint64_t timestamp;
};

/**
 * @brief Structure to handle temperature data.
 */
struct TemperatureData
{
    uint64_t temp_timestamp;
    float temp;
};

/**
 * @brief Structure to handle pressure data.
 */
struct PressureData
{
    uint64_t press_timestamp;
    float press;
};

/**
 * @brief Structure to handle accelerometer data.
 */
struct AccelerometerData
{
    uint64_t accel_timestamp;
    float accel_x;
    float accel_y;
    float accel_z;
};

/**
 * @brief Structure to handle gyroscope data.
 */
struct GyroscopeData
{
    uint64_t gyro_timestamp;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

/**
 * @brief Structure to handle magnetometer data.
 */
struct MagnetometerData
{
    uint64_t mag_timestamp;
    float mag_x;
    float mag_y;
    float mag_z;
};

/**
 * @brief Structure to handle GPS data.
 */
struct GPSData
{
    uint64_t gps_timestamp;
    float latitude;         /**< [deg] */
    float longitude;        /**< [deg] */
    float height;           /**< [m]   */
    float velocity_north;   /**< [m/s] */
    float velocity_east;    /**< [m/s] */
    float velocity_down;    /**< [m/s] */
    float speed;            /**< [m/s] */
    uint8_t num_satellites; /**< [1]   */
    bool fix;
};

/**
 * @brief Structure to handle ADC data.
 */
struct ADCData
{
    uint64_t adc_timestamp;
    float voltage;
};