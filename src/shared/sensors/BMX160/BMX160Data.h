/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Davide Mor
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

struct BMX160Data : public AccelerometerData,
                    public GyroscopeData,
                    public MagnetometerData
{
    BMX160Data()
        : AccelerometerData{0, 0.0, 0.0, 0.0}, GyroscopeData{0, 0.0, 0.0, 0.0},
          MagnetometerData{0, 0.0, 0.0, 0.0}
    {
    }

    BMX160Data(AccelerometerData acc, GyroscopeData gyr, MagnetometerData mag)
        : AccelerometerData(acc), GyroscopeData(gyr), MagnetometerData(mag)
    {
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(BMX160Data, EXTEND_DEF(AccelerometerData)
                                          EXTEND_DEF(GyroscopeData)
                                              EXTEND_DEF(MagnetometerData));
    }
};
struct BMX160Temperature : public TemperatureData
{
    static constexpr auto reflect()
    {
        return STRUCT_DEF(BMX160Temperature, EXTEND_DEF(TemperatureData));
    }
};

/**
 * @brief BMX160 fifo statistics.
 */
struct BMX160FifoStats
{
    uint64_t timestamp;

    uint64_t watermarkTimestamp;  ///< Watermark timestamp (from the start of
                                  // the fifo)
    uint64_t fifoDuration;        ///< Total fifo duration
    uint64_t interruptTimestampDelta;  ///< Reported delta time between the
                                       ///< previous
                                       // interrupt and the current one.
    int len;                           ///< Fifo length in bytes.

    static constexpr auto reflect()
    {
        return STRUCT_DEF(BMX160FifoStats,
                          FIELD_DEF(timestamp) FIELD_DEF(watermarkTimestamp)
                              FIELD_DEF(fifoDuration)
                                  FIELD_DEF(interruptTimestampDelta)
                                      FIELD_DEF(len));
    }
};

}  // namespace Boardcore
