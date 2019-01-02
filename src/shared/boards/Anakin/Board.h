/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BOARD_H
#define BOARD_H

#include <Common.h>
#include <Singleton.h>
#include <drivers/BusTemplate.h>
#include <sensors/Sensor.h>

enum DataType
{
    DATA_VEC3  = 0,
    DATA_QUAT  = 1,
    DATA_FLOAT = 2,
    DATA_INT   = 3,
};

struct SingleSensor
{
    uint16_t sensor;
    DataType data;
    const void* value;

    SingleSensor() {}
    SingleSensor(uint16_t sensor, DataType data, const void* value)
        : sensor(sensor), data(data), value(value)
    {
    }
};

class Board
{
public:
    Board() : mInited(false) {}
    virtual bool init() = 0;

protected:
    bool mInited;
    std::vector<SingleSensor> mSensorData;
    std::vector<Sensor*> mRawSensors;

    void AddSensor(uint16_t sensor, DataType data, const void* value)
    {
        mSensorData.push_back(SingleSensor(sensor, data, value));
    }
};

#endif /* BOARD_H */
