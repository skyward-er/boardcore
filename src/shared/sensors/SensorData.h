/* Copyright (c) 2020-2022 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Alberto Nidasio
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
    uint64_t loadTimestamp = 0;
    float load             = 0;

    static std::string header() { return "loadTimestamp,load\n"; }

    void print(std::ostream& os) const
    {
        os << loadTimestamp << "," << load << "\n";
    }
};

struct TemperatureData
{
    uint64_t temperatureTimestamp = 0;
    float temperature             = 0;

    static std::string header() { return "timestamp,temperature\n"; }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << "\n";
    }
};

struct PressureData
{
    uint64_t pressureTimestamp = 0;
    float pressure             = 0;

    static std::string header() { return "timestamp,pressure\n"; }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << "\n";
    }
};

/**
 * @brief Structure to handle humidity data.
 */
struct HumidityData
{
    uint64_t humidityTimestamp = 0;
    float humidity             = 0;
};

/**
 * @brief Structure to handle accelerometer data.
 */
struct AccelerometerData
{
    uint64_t accelerationTimestamp = 0;
    float accelerationX            = 0;
    float accelerationY            = 0;
    float accelerationZ            = 0;

    static std::string header()
    {
        return "timestamp,accelerationX,accelerationY,accelerationZ\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << "\n";
    }
};

/**
 * @brief Structure to handle gyroscope data.
 */
struct GyroscopeData
{
    uint64_t angularVelocityTimestamp = 0;
    float angularVelocityX            = 0;
    float angularVelocityY            = 0;
    float angularVelocityZ            = 0;

    static std::string header()
    {
        return "timestamp,angularVelocityX,angularVelocityY,angularVelocityZ\n";
    }

    void print(std::ostream& os) const
    {
        os << angularVelocityTimestamp << "," << angularVelocityX << ","
           << angularVelocityY << "," << angularVelocityZ << "\n";
    }
};

/**
 * @brief Structure to handle magnetometer data.
 */
struct MagnetometerData
{
    uint64_t magneticFieldTimestamp = 0;
    float magneticFieldX            = 0;
    float magneticFieldY            = 0;
    float magneticFieldZ            = 0;

    static std::string header()
    {
        return "timestamp,magneticFieldX,magneticFieldY,magneticFieldZ\n";
    }

    void print(std::ostream& os) const
    {
        os << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
    }
};

/**
 * @brief Structure to handle GPS data.
 */
struct GPSData
{
    uint64_t gpsTimestamp = 0;
    float latitude        = 0;  // [deg]
    float longitude       = 0;  // [deg]
    float height          = 0;  // [m]
    float velocityNorth   = 0;  // [m/s]
    float velocityEast    = 0;  // [m/s]
    float velocityDown    = 0;  // [m/s]w
    float speed           = 0;  // [m/s]
    float track           = 0;  // [deg]
    float positionDOP     = 0;  // [?]
    uint8_t satellites    = 0;  // [1]
    uint8_t fix           = 0;  // 0 = no fix

    static std::string header()
    {
        return "timestamp,latitude,longitude,height,velocityNorth,velocityEast,"
               "velocityDown,speed,track,positionDOP,satellites,fix\n";
    }

    void print(std::ostream& os) const
    {
        os << gpsTimestamp << "," << longitude << "," << latitude << ","
           << height << "," << velocityNorth << "," << velocityEast << ","
           << velocityDown << "," << speed << "," << track << "," << positionDOP
           << "," << (int)satellites << "," << (int)fix << "\n";
    }
};

/**
 * @brief Structure to handle ADC data.
 */
struct ADCData
{
    uint64_t voltageTimestamp = 0;
    uint8_t channelId         = 0;
    float voltage             = 0;
};

}  // namespace Boardcore
