/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <drivers/timer/TimestampTimer.h>
#include <sensors/SensorData.h>

class ValueFollower
{
public:
    ValueFollower(Boardcore::PressureData *data, size_t length,
                  uint64_t timestampOffset = 0, float pressureOffset = 0)
        : data(data), length(length), timestampOffset(timestampOffset),
          pressureOffset(pressureOffset)
    {
        dataDuration =
            data[length - 1].pressureTimestamp - data[0].pressureTimestamp;
    }

    /**
     * @brief Find the nearest data point with the specified timestamp.
     */
    float findNext(
        uint64_t timestamp = Boardcore::TimestampTimer::getTimestamp())
    {
        timestamp = timestamp % dataDuration;
        if (data[currentIndex].pressureTimestamp + timestampOffset > timestamp)
            return data[currentIndex].pressure.value() + pressureOffset;

        for (; currentIndex < length; currentIndex++)
            if (data[currentIndex + 1].pressureTimestamp + timestampOffset >
                timestamp)
                return data[currentIndex].pressure.value() + pressureOffset;

        return data[length - 1].pressure.value() + pressureOffset;
    }

    size_t getSize() { return length; }

    u_int64_t getDataDuration() { return dataDuration; }

private:
    Boardcore::PressureData *data;
    size_t length;

    uint64_t timestampOffset;
    float pressureOffset;
    uint64_t dataDuration;  // Duration of reference data

    size_t currentIndex = 0;
};
