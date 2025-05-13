/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Fabrizio Monti
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

#include <reflect.hpp>

namespace Boardcore
{

/**
 * @brief data type class
 */
struct VN100SerialData : public QuaternionData,
                         public MagnetometerData,
                         public AccelerometerData,
                         public GyroscopeData,
                         public TemperatureData,
                         public PressureData
{
    /**
     * @brief Void parameters constructor
     */
    // cppcheck-suppress uninitDerivedMemberVar
    VN100SerialData()
        : QuaternionData{0, 0.0, 0.0, 0.0, 0.0},
          MagnetometerData{0, 0.0, 0.0, 0.0},
          AccelerometerData{0, 0.0, 0.0, 0.0}, GyroscopeData{0, 0.0, 0.0, 0.0},
          TemperatureData{0, 0.0}, PressureData{0, 0.0}
    {
    }

    /**
     * @brief Constructor with parameters
     *
     * @param single data structures for all the data
     */
    // cppcheck-suppress passedByValue
    // cppcheck-suppress uninitDerivedMemberVar
    VN100SerialData(QuaternionData quat, MagnetometerData magData,
                    AccelerometerData accData, GyroscopeData gyro,
                    TemperatureData temp, PressureData pres)
        : QuaternionData(quat), MagnetometerData(magData),
          AccelerometerData(accData), GyroscopeData(gyro),
          TemperatureData(temp), PressureData(pres)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            VN100SerialData,
            EXTEND_DEF(QuaternionData) EXTEND_DEF(MagnetometerData)
                EXTEND_DEF(AccelerometerData) EXTEND_DEF(GyroscopeData)
                    EXTEND_DEF(TemperatureData) EXTEND_DEF(PressureData));
    }
};

}  // namespace Boardcore
