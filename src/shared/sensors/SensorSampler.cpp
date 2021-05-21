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
// #include <diagnostic/PrintLogger.h>

using namespace std;

SensorSampler::SensorSampler(uint8_t id, uint32_t period, bool is_dma)
    : id(id), period(period), is_dma(is_dma)
{
}

SensorSampler::~SensorSampler() { sensors.clear(); }

void SensorSampler::sampleAndCallback()
{
    for (auto& s : sensors)
    {
        // sample only if that sensor is enabled
        if (s.second.is_enabled)
        {
            // PrintLogger log = Logging::getLogger("ssampler");
            
            sampleSensor(s.first);
            s.second.callback();
        }
    }
}

void SensorSampler::toggleSensor(AbstractSensor* sensor, bool is_en)
{
    for (auto& s : sensors)
    {
        if (s.first == sensor)
        {
            s.second.is_enabled = is_en;
            TRACE("[Sampler %d] Toggle Sensor %p ---> enabled = %d \n", getID(),
                  sensor, s.second.is_enabled);
            break;
        }
    }
}

void SensorSampler::enableAllSensors()
{
    for (auto& s : sensors)
    {
        s.second.is_enabled = true;
    }
}

void SensorSampler::disableAllSensors()
{
    for (auto& s : sensors)
    {
        s.second.is_enabled = false;
    }
}

bool SensorSampler::isDMA() { return is_dma; }

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

    TRACE("[SM] Sampler %d : sensor %p not found \n", this->id, sensor);
    
    return SensorInfo{};
}

// simple sampler
SimpleSensorSampler::SimpleSensorSampler(uint8_t id, uint32_t period)
    : SensorSampler(id, period, false)
{
}

SimpleSensorSampler::~SimpleSensorSampler() {}

void SimpleSensorSampler::addSensor(AbstractSensor* sensor,
                                    SensorInfo sensor_info)
{
    sensors.push_back(make_pair(sensor, sensor_info));
}

void SimpleSensorSampler::sampleSensor(AbstractSensor* sensor)
{
    sensor->sample();
}

// DMA sampler
DMASensorSampler::DMASensorSampler(uint8_t id, uint32_t period)
    : SensorSampler(id, period, true)
{
}

DMASensorSampler::~DMASensorSampler() {}

void DMASensorSampler::addSensor(AbstractSensor* sensor, SensorInfo sensor_info)
{
    sensors.push_back(make_pair(sensor, sensor_info));
}

/*
    TODO : Handle sensors that use DMA
*/
void DMASensorSampler::sampleSensor(AbstractSensor* sensor)
{
    sensor->sample();
}