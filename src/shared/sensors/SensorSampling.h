/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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
#ifndef SENSOR_SAMPLING_H
#define SENSOR_SAMPLING_H

#include <Common.h>
#include <diagnostic/Log.h>
#include <drivers/spi/SensorSpi.h>

#include "Sensor.h"

class DMASensorSampler
{
public:
    DMASensorSampler() {}
    ~DMASensorSampler() {}

    void AddSensor(Sensor* sensor)
    {
        std::vector<SPIRequest> requests = sensor->buildDMARequest();
        for (size_t i = 0; i < requests.size(); i++)
        {
            mRequests.push_back(requests[i]);
            mSensors.push_back(sensor);
        }
    }

    void Update(std::function<void()> onSampleUpdateCallback)
    {
        auto& driver = SPIDriver::instance();
        bool ret     = driver.transaction(mRequests);

        if (ret)
            for (size_t i = 0; i < mSensors.size(); i++)
                mSensors[i]->onDMAUpdate(mRequests[i]);

        onSampleUpdateCallback();
    }

private:
    std::vector<Sensor*> mSensors;
    std::vector<SPIRequest> mRequests;
};

class SimpleSensorSampler
{
public:
    SimpleSensorSampler() {}
    ~SimpleSensorSampler() {}

    void AddSensor(Sensor* sensor) { mSensors.push_back(sensor); }

    void Update(std::function<void()> onSampleUpdateCallback)
    {
        for (Sensor* s : mSensors)
            s->onSimpleUpdate();

        onSampleUpdateCallback();
    }

private:
    std::vector<Sensor*> mSensors;
};

#endif /* ifndef SENSOR_SAMPLING_H */
