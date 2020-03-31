/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include "SensorManager.h"

SensorManager::SensorManager() {
    
}

SensorManager::~SensorManager() {
    
}

bool SensorManager::start() {
    initScheduler();
    return scheduler.start();
}

void SensorManager::stop() {
    scheduler.stop();
}

bool SensorManager::addSensor(Sensor* sensor, 
                                uint32_t freq, 
                                function_t sensor_callback, 
                                SamplerType sampler_type) {

    // avoid adding sensors that fail to be initalized
    if (!initSensor(sensor)) {
        TRACE("[SM] Failed to initialize sensor \n");
        return false;
    }

    TRACE("[SM] Adding sensor with frequency %u Hz \n", freq);

    // check if a sampler with the same frequency and the same type exists
    for(auto s : samplers) {
        if (freq == s.sampler->getFrequency() &&
                sampler_type == s.sampler->getType()) { 
            s.sampler->addSensor(sensor, sensor_callback);
            return true;
        }
    }

    SensorSampler* new_sampler;

    if (sampler_type == SIMPLE_SAMPLER)  // assign the correct type and frequency to the sampler
        new_sampler = new SimpleSensorSampler(freq, current_id);
    else
        new_sampler = new DMASensorSampler(freq, current_id);

    current_id++;

    new_sampler->addSensor(sensor, sensor_callback);  // add the sensor to the sampler

    /*
     * std::bind syntax:
     * std::bind(&MyClass::someFunction, &myclass_instance, [someFunction args])
     */
    // function to be periodically called by the scheduler (to sample the sensors)
    function_t new_sampler_update_function = std::bind(&SensorSampler::sampleAndCallback, new_sampler);

    samplers.push_back({
        new_sampler,
        new_sampler_update_function
    });

    return true;
}

void SensorManager::addCallback(uint32_t freq, function_t callback, uint8_t id) {
    scheduler.add(
        callback,
        1000 / freq,
        id, 
        miosix::getTick() + 10
    );
}

bool SensorManager::initSensor(Sensor* sensor) {
    return sensor->init() && sensor->selfTest();
}

void SensorManager::initScheduler() {
    uint64_t start_time = miosix::getTick() + 10;
    uint32_t period = 0;
    // add all the samplers to the scheduler
    for(auto& s : samplers) {
        period = 1000 / s.sampler->getFrequency(); // in milliseconds
        // use the frequency as the ID of the task in the scheduler
        scheduler.add(s.sampler_update_function, period, s.sampler->getId(), start_time);
    }
}

vector<SensorManager::Sampler_t>& SensorManager::getSamplers() {
    return samplers;
}

TaskScheduler& SensorManager::getScheduler() { 
    return scheduler; 
}
