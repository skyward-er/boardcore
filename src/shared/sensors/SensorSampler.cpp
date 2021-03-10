/* Copyright (c) 2017-2020 Skyward Experimental Rocketry
 * Author: Alain Carlucci, Luca Conterio
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

#include "SensorSampler.h"

using namespace std;

SensorSampler::SensorSampler(uint8_t id, uint32_t freq, bool is_dma)
    : id(id), freq(freq), is_dma(is_dma)
{
}

SensorSampler::~SensorSampler()
{
    sensors_map.clear();
}

void SensorSampler::sampleAndCallback()
{
    for (auto it = sensors_map.begin(); it != sensors_map.end(); it++)
    {
        // sample only if that sensor is enabled
        if (it->second.is_enabled)
        {
            sampleSensor(it->first);
            it->second.callback();
        }
    }
}

void SensorSampler::toggleSensor(AbstractSensor* sensor, bool is_en)
{
    sensors_map.at(sensor).is_enabled = is_en;

    TRACE("[Sampler %d] Toggle Sensor %p, Sensor info %p ---> enabled = %d \n",
          getID(), sensor, sensors_map.at(sensor),
          sensors_map.at(sensor).is_enabled.load());
}

bool SensorSampler::isDMA() { return is_dma; }

uint8_t SensorSampler::getID() { return id; }

uint32_t SensorSampler::getFrequency() { return freq; }

uint32_t SensorSampler::getNumSensors() { return sensors_map.size(); }

const SensorInfo& SensorSampler::getSensorInfo(AbstractSensor* sensor)
{
    return sensors_map.at(sensor);
}

// simple sampler
SimpleSensorSampler::SimpleSensorSampler(uint8_t id, uint32_t freq)
    : SensorSampler(id, freq, false)
{
}

SimpleSensorSampler::~SimpleSensorSampler() {}

void SimpleSensorSampler::addSensor(AbstractSensor* sensor,
                                    SensorInfo sensor_info)
{
    sensors_map.emplace(sensor, sensor_info);

    TRACE("[Sampler %d] Added : Sensor %p, Sensor info %p ---> enabled = %d\n",
          getID(), sensor, &sensors_map.at(sensor),
          sensors_map.at(sensor).is_enabled.load());
}

void SimpleSensorSampler::sampleSensor(AbstractSensor* sensor)
{
    sensor->sample();
}

// DMA sampler
DMASensorSampler::DMASensorSampler(uint8_t id, uint32_t freq)
    : SensorSampler(id, freq, true)
{
}

DMASensorSampler::~DMASensorSampler() {}

void DMASensorSampler::addSensor(AbstractSensor* sensor, SensorInfo sensor_info)
{
    sensors_map.emplace(sensor, sensor_info);

    TRACE("[Sampler %d] Added : Sensor %p, Sensor info %p ---> enabled = %d\n",
          this->getID(), sensor, &sensors_map.at(sensor),
          sensors_map.at(sensor).is_enabled.load());
}

/*
    TBD
    Handle sensors that use DMA
*/
void DMASensorSampler::sampleSensor(AbstractSensor* sensor)
{
    sensor->sample();
}