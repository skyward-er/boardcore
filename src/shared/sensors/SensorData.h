/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <ostream>

namespace Boardcore
{

/**
 * @brief Generic error codes that a sensor can generate.
 *
 * Sensors can extend this enum by defining a new set of errors,
 * starting from END_OF_BASE_ERRORS.
 */
enum SensorErrors : uint8_t
{
    NO_ERRORS          = 0,
    INVALID_WHOAMI     = 1,
    INIT_FAIL          = 2,
    NOT_INIT           = 3,  // if some method called before init()
    ALREADY_INIT       = 4,  // if init() called multiple times
    SELF_TEST_FAIL     = 5,
    BUS_FAULT          = 6,
    NO_NEW_DATA        = 7,  // no new data available from the sensor
    INVALID_FIFO_INDEX = 8,
    DMA_ERROR          = 9,
    COMMAND_FAILED     = 10,
    END_OF_BASE_ERRORS = 11  // used to extend this enum
};

struct TimestampData
{
    uint64_t timestamp;
};

struct LoadCellData
{
    uint64_t weightTimestamp;
    float weight;
};

struct TemperatureData
{
    uint64_t temperatureTimestamp;
    float temperature;

    static std::string header() { return "timestamp,temperature\n"; }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << "\n";
    }
};

struct PressureData
{
    uint64_t pressureTimestamp;
    float pressure;
};

/**
 * @brief Structure to handle humidity data.
 */
struct HumidityData
{
    uint64_t humidityTimestamp;
    float humidity;
};

/**
 * @brief Structure to handle accelerometer data.
 */
struct AccelerometerData
{
    uint64_t accelerationTimestamp;
    float accelerationX;
    float accelerationY;
    float accelerationZ;
};

/**
 * @brief Structure to handle gyroscope data.
 */
struct GyroscopeData
{
    uint64_t angularVelocityTimestamp;
    float angularVelocityX;
    float angularVelocityY;
    float angularVelocityZ;
};

/**
 * @brief Structure to handle magnetometer data.
 */
struct MagnetometerData
{
    uint64_t magneticFieldTimestamp;
    float magneticFieldX;
    float magneticFieldY;
    float magneticFieldZ;
};

/**
 * @brief Structure to handle GPS data.
 */
struct GPSData
{
    uint64_t gpsTimestamp;
    float latitude;      /**< [deg] */
    float longitude;     /**< [deg] */
    float height;        /**< [m]   */
    float velocityNorth; /**< [m/s] */
    float velocityEast;  /**< [m/s] */
    float velocityDown;  /**< [m/s] */
    float speed;         /**< [m/s] */
    float track;         /**< [deg] */
    uint8_t satellites;  /**< [1]   */
    bool fix;
};

/**
 * @brief Structure to handle ADC data.
 */
struct ADCData
{
    uint64_t voltageTimestamp;
    uint8_t channelId;
    float voltage;
};

}  // namespace Boardcore
