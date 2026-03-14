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

#include <Eigen/Core>
#include <ostream>
#include <reflect.hpp>

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

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TimestampData, FIELD_DEF_WITH_UNIT(timestamp, microseconds));
    }
};

struct LoadCellData
{
    uint64_t loadTimestamp = 0;
    float load             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(LoadCellData,
                          FIELD_DEF_WITH_UNIT(loadTimestamp, microseconds) FIELD_DEF_WITH_UNIT(load, N));
    }
};

struct TemperatureData
{
    uint64_t temperatureTimestamp = 0;
    float temperature             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(TemperatureData, FIELD_DEF_WITH_UNIT(temperatureTimestamp, microseconds)
                                               FIELD_DEF_WITH_UNIT(temperature, °C));
    }
};

struct PressureData
{
    uint64_t pressureTimestamp = 0;
    float pressure             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(PressureData,
                          FIELD_DEF_WITH_UNIT(pressureTimestamp, microseconds) FIELD_DEF_WITH_UNIT(pressure, hPa));
    }
};

/**
 * @brief Structure to handle humidity data.
 */
struct HumidityData
{
    uint64_t humidityTimestamp = 0;
    float humidity             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(HumidityData,
                          FIELD_DEF_WITH_UNIT(humidityTimestamp, microseconds) FIELD_DEF_WITH_UNIT(humidity, %RE));
    }
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

    AccelerometerData() {}

    AccelerometerData(uint64_t timestamp, float x, float y, float z)
        : accelerationTimestamp(timestamp), accelerationX(x), accelerationY(y),
          accelerationZ(z)
    {
    }

    AccelerometerData(const AccelerometerData& data) = default;

    explicit AccelerometerData(const Eigen::Vector3f& acc)
        : accelerationX(acc(0)), accelerationY(acc(1)), accelerationZ(acc(2))
    {
    }

    operator Eigen::Vector3f() const
    {
        return {accelerationX, accelerationY, accelerationZ};
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(AccelerometerData,
                          FIELD_DEF_WITH_UNIT(accelerationTimestamp, microseconds)
                              FIELD_DEF_WITH_UNIT(accelerationX, g) FIELD_DEF_WITH_UNIT(accelerationY, g)
                                  FIELD_DEF_WITH_UNIT(accelerationZ, g));
    }
};

/**
 * @brief Structure to handle quaternion data.
 */
struct QuaternionData
{
    uint64_t quaternionTimestamp = 0;
    float quaternionX            = 0;
    float quaternionY            = 0;
    float quaternionZ            = 0;
    float quaternionW            = 0;

    QuaternionData() {}

    QuaternionData(uint64_t timestamp, float x, float y, float z, float w)
        : quaternionTimestamp(timestamp), quaternionX(x), quaternionY(y),
          quaternionZ(z), quaternionW(w)
    {
    }

    QuaternionData(const QuaternionData& data) = default;

    explicit QuaternionData(const Eigen::Vector4f& quat)
        : quaternionX(quat(0)), quaternionY(quat(1)), quaternionZ(quat(2)),
          quaternionW(quat(3))
    {
    }

    operator Eigen::Vector4f() const
    {
        return {quaternionX, quaternionY, quaternionZ, quaternionW};
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(QuaternionData,
                          FIELD_DEF_WITH_UNIT(quaternionTimestamp, microseconds) FIELD_DEF(quaternionX)
                              FIELD_DEF(quaternionY) FIELD_DEF(quaternionZ)
                                  FIELD_DEF(quaternionW));
    }
};

/**
 * @brief Structure to handle gyroscope data.
 */
struct GyroscopeData
{
    uint64_t angularSpeedTimestamp = 0;
    float angularSpeedX            = 0;
    float angularSpeedY            = 0;
    float angularSpeedZ            = 0;

    GyroscopeData() {}

    GyroscopeData(uint64_t timestamp, float x, float y, float z)
        : angularSpeedTimestamp(timestamp), angularSpeedX(x), angularSpeedY(y),
          angularSpeedZ(z)
    {
    }

    GyroscopeData(const GyroscopeData& data) = default;

    explicit GyroscopeData(const Eigen::Vector3f& vel)
        : angularSpeedX(vel(0)), angularSpeedY(vel(1)), angularSpeedZ(vel(2))
    {
    }

    operator Eigen::Vector3f() const
    {
        return {angularSpeedX, angularSpeedY, angularSpeedZ};
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(GyroscopeData,
                          FIELD_DEF_WITH_UNIT(angularSpeedTimestamp, microseconds)
                              FIELD_DEF_WITH_UNIT(angularSpeedX, deg/sec) FIELD_DEF_WITH_UNIT(angularSpeedY, deg/sec)
                                  FIELD_DEF_WITH_UNIT(angularSpeedZ, deg/sec));
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

    MagnetometerData() {}

    MagnetometerData(uint64_t timestamp, float x, float y, float z)
        : magneticFieldTimestamp(timestamp), magneticFieldX(x),
          magneticFieldY(y), magneticFieldZ(z)
    {
    }

    MagnetometerData(const MagnetometerData& data) = default;

    explicit MagnetometerData(const Eigen::Vector3f& mag)
        : magneticFieldX(mag(0)), magneticFieldY(mag(1)), magneticFieldZ(mag(2))
    {
    }

    operator Eigen::Vector3f() const
    {
        return {magneticFieldX, magneticFieldY, magneticFieldZ};
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            MagnetometerData,
            FIELD_DEF_WITH_UNIT(magneticFieldTimestamp, microseconds) FIELD_DEF_WITH_UNIT(magneticFieldX, microTesla)
                FIELD_DEF_WITH_UNIT(magneticFieldY, microTesla) FIELD_DEF_WITH_UNIT(magneticFieldZ, microTesla));
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

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            GPSData,
            FIELD_DEF_WITH_UNIT(gpsTimestamp, microseconds) FIELD_DEF_WITH_UNIT(latitude, degrees) FIELD_DEF_WITH_UNIT(longitude, degrees)
                FIELD_DEF_WITH_UNIT(height, meters) FIELD_DEF_WITH_UNIT(velocityNorth, m/s)
                    FIELD_DEF_WITH_UNIT(velocityEast, m/s) FIELD_DEF_WITH_UNIT(velocityDown, m/s)
                        FIELD_DEF_WITH_UNIT(speed, m/s) FIELD_DEF(track) FIELD_DEF(positionDOP)
                            FIELD_DEF(satellites) FIELD_DEF(fix));
    }
};

/**
 * @brief Structure to handle current data.
 */
struct CurrentData
{
    uint64_t currentTimestamp = 0;
    float current             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CurrentData,
                          FIELD_DEF_WITH_UNIT(currentTimestamp, microseconds) FIELD_DEF_WITH_UNIT(current, A));
    }
};

/**
 * @brief Structure to handle voltage data.
 */
struct VoltageData
{
    uint64_t voltageTimestamp = 0;
    float voltage             = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(VoltageData,
                          FIELD_DEF_WITH_UNIT(voltageTimestamp, microseconds) FIELD_DEF_WITH_UNIT(voltage, V));
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

    /**
     * @brief Allows implicit conversions to VoltageData.
     */
    operator VoltageData() const
    {
        return VoltageData{
            .voltageTimestamp = voltageTimestamp,
            .voltage          = voltage,
        };
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ADCData, FIELD_DEF_WITH_UNIT(voltageTimestamp, microseconds)
                                       FIELD_DEF(channelId) FIELD_DEF_WITH_UNIT(voltage, V));
    }
};

}  // namespace Boardcore
