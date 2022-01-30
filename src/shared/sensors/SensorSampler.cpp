/* Copyright (c) 2017-2020 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Luca Conterio
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

#include "SensorSampler.h"
// #include <diagnostic/PrintLogger.h>

using namespace std;

namespace Boardcore
{

SensorSampler::SensorSampler(uint8_t id, uint32_t period)
    : id(id), period(period)
{
}

SensorSampler::~SensorSampler() { sensors.clear(); }

bool SensorSampler::comparareByPeriod(SensorSampler* left, SensorSampler* right)
{
    return left->getSamplingPeriod() < right->getSamplingPeriod();
}

void SensorSampler::toggleSensor(AbstractSensor* sensor, bool isEnabled)
{
    auto elem = std::find_if(sensors.begin(), sensors.end(),
                             [&](std::pair<AbstractSensor*, SensorInfo> s)
                             { return s.first == sensor; });

    elem->second.isEnabled = isEnabled;
    LOG_DEBUG(logger, "Sampler {}, toggle Sensor {} ---> enabled = {}", getID(),
              static_cast<void*>(sensor), elem->second.isEnabled);
}

void SensorSampler::enableAllSensors()
{
    for (auto& s : sensors)
    {
        s.second.isEnabled = true;
    }
}

void SensorSampler::disableAllSensors()
{
    for (auto& s : sensors)
    {
        s.second.isEnabled = false;
    }
}

void SensorSampler::sampleAndCallback()
{
    for (auto& s : sensors)
    {
        // Sample only if that sensor is enabled and initialized
        if (s.second.isEnabled && s.second.isInitialized)
        {
            sampleSensor(s.first);

            if (s.second.callback)
                s.second.callback();
        }
    }
}

uint8_t SensorSampler::getID() { return id; }

uint32_t SensorSampler::getSamplingPeriod() { return period; }

unsigned int SensorSampler::getNumSensors() { return sensors.size(); }

const SensorInfo SensorSampler::getSensorInfo(AbstractSensor* sensor)
{
    for (auto it = sensors.begin(); it != sensors.end(); ++it)
    {
        if (it->first == sensor)
        {
            return it->second;
        }
    }

    LOG_ERR(logger, "Sampler {}: sensor {} not found", this->id,
            static_cast<void*>(sensor));

    return SensorInfo{};
}

SimpleSensorSampler::SimpleSensorSampler(uint8_t id, uint32_t period)
    : SensorSampler(id, period)
{
}

SimpleSensorSampler::~SimpleSensorSampler() {}

void SimpleSensorSampler::addSensor(AbstractSensor* sensor,
                                    SensorInfo sensorInfo)
{
    sensors.push_back(make_pair(sensor, sensorInfo));
}

void SimpleSensorSampler::sampleSensor(AbstractSensor* sensor)
{
    sensor->sample();
}

}  // namespace Boardcore
