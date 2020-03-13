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

SensorSampler::SensorSampler(SamplerType type, uint32_t freq, uint32_t id) : 
                        type(type), freq(freq), id(id) {}

void SensorSampler::sampleAndCallback() {   
    for (auto it = sensors_map.begin(); it != sensors_map.end(); it++) {
        sampleSensor(it->first);
        it->second();  // callback
    }
}

SamplerType SensorSampler::getType() {
    return type;
}

uint32_t SensorSampler::getId() {
    return id;
}

uint32_t SensorSampler::getFrequency() {
    return freq;
}

uint32_t SensorSampler::getNumSensors() {
    return sensors_map.size();
}

// simple sampler

SimpleSensorSampler::SimpleSensorSampler(uint32_t freq, uint32_t id) : 
                        SensorSampler(SIMPLE_SAMPLER, freq, id) {}

SimpleSensorSampler::~SimpleSensorSampler() {}

void SimpleSensorSampler::addSensor(Sensor* sensor, function_t sensor_callback) {
    sensors_map[sensor] = sensor_callback; 
}

void SimpleSensorSampler::sampleSensor(Sensor* s) {
    s->onSimpleUpdate();
}

// DMA sampler
DMASensorSampler::DMASensorSampler(uint32_t freq, uint32_t id) : 
                    SensorSampler(DMA_SAMPLER, freq, id) {}


DMASensorSampler::~DMASensorSampler() {}

void DMASensorSampler::addSensor(Sensor* sensor, function_t sensor_callback) {
    vector<SPIRequest> requests = sensor->buildDMARequest();
    
    sensors_map[sensor] = sensor_callback;
    requests_map.insert(pair<Sensor*, vector<SPIRequest>>(sensor, requests)); // can't use standard operator=
}

void DMASensorSampler::sampleSensor(Sensor* s) {
    auto& driver = SPIDriver::instance();
    
    vector<SPIRequest> requests = requests_map[s];

    if (driver.transaction(requests)) {
        for (auto r : requests) {
            s->onDMAUpdate(r);
        }
    }
}