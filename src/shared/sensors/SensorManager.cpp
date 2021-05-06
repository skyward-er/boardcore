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

SensorManager::SensorManager(const SensorMap_t& sensors_map)
    : scheduler(new TaskScheduler())
{
    if (!init(sensors_map))
    {
        TRACE("[SM] Initialization failed \n");
    }
}

SensorManager::SensorManager(TaskScheduler* scheduler,
                             const SensorMap_t& sensors_map)
    : scheduler(scheduler)
{
    if (!init(sensors_map))
    {
        TRACE("[SM] Initialization failed \n");
    }
}

SensorManager::~SensorManager()
{
    for (auto s : samplers)
    {
        delete s;
    }
}

bool SensorManager::start() { return scheduler->start(); }

void SensorManager::stop() { scheduler->stop(); }

void SensorManager::enableSensor(AbstractSensor* sensor)
{
    samplers_map[sensor]->toggleSensor(sensor, true);
}

void SensorManager::disableSensor(AbstractSensor* sensor)
{
    samplers_map[sensor]->toggleSensor(sensor, false);
}

void SensorManager::enableAllSensors()
{
    for (auto s : samplers)
    {
        s->enableAllSensors();
    }
}

void SensorManager::disableAllSensors()
{
    for (auto s : samplers)
    {
        s->disableAllSensors();
    }
}

const SensorInfo& SensorManager::getSensorInfo(AbstractSensor* sensor)
{
    return samplers_map[sensor]->getSensorInfo(sensor);
}

const vector<TaskStatResult> SensorManager::getSamplersStats()
{
    return scheduler->getTaskStats();
}

bool SensorManager::init(const SensorMap_t& sensors_map)
{
    bool init_result           = true;
    uint8_t current_sampler_id = getFirstTaskID();

    if (current_sampler_id != 0)
    {
        TRACE("[SM] Task scheduler not empty : starting from task ID %u \n",
              current_sampler_id);
    }

    for (auto it = sensors_map.begin(); it != sensors_map.end(); it++)
    {
        AbstractSensor* sensor = it->first;
        SensorInfo sensor_info = it->second;

        TRACE("[SM] Sensor %p, Sensor info %p ---> enabled = %d \n", sensor,
              sensor_info, sensor_info.is_enabled);

        // avoid adding sensors that fail to be initalized
        if (!initSensor(sensor))
        {
            init_result = false;
            TRACE("[SM] Failed to initialize sensor ---> Error : %u \n",
                  sensor->getLastError());
        }
        else
        {
            TRACE("[SM] Adding sensor with sampling period %u ms \n",
                  sensor_info.period);

            // check if a sampler with the same sampling period and the same
            // type exists
            bool found = false;
            for (auto s : samplers)
            {
                if (sensor_info.period == s->getSamplingPeriod() &&
                    sensor_info.is_dma == s->isDMA())
                {
                    s->addSensor(sensor, sensor_info);
                    samplers_map[sensor] = s;
                    found                = true;
                }
            }

            if (!found)
            {
                // a sampler with the required frequency does not exist yet
                SensorSampler* new_sampler = createSampler(
                    current_sampler_id, sensor_info.period, sensor_info.is_dma);

                new_sampler->addSensor(sensor, sensor_info);

                samplers.push_back(new_sampler);
                samplers_map[sensor] = new_sampler;

                if (current_sampler_id == MAX_TASK_ID)
                {
                    TRACE(
                        "[SM] WARNING : Max task ID (255) reached in task "
                        "scheduler, IDs will start again from 0 \n");
                }

                current_sampler_id++;
            }
        }
    }

    initScheduler();

    return init_result;
}

bool SensorManager::initSensor(AbstractSensor* sensor)
{
    return sensor->init() && sensor->selfTest();
}

void SensorManager::initScheduler()
{
    uint64_t start_time = miosix::getTick() + 10;
    uint32_t period     = 0;
    // add all the samplers to the scheduler
    for (auto& s : samplers)
    {
        function_t sampler_update_function =
            bind(&SensorSampler::sampleAndCallback, s);

        scheduler->add(sampler_update_function, s->getSamplingPeriod(),
                       s->getID(), start_time);
    }
}

uint8_t SensorManager::getFirstTaskID()
{
    std::vector<TaskStatResult> tasks_stats = scheduler->getTaskStats();

    if (tasks_stats.empty())
    {
        return 0;
    }

    uint8_t max_id = 0;
    for (auto& stats : tasks_stats)
    {
        if (stats.id > max_id)
        {
            max_id = stats.id;
        }
    }

    return max_id + 1;
}

SensorSampler* SensorManager::createSampler(uint8_t id, uint32_t period,
                                            bool is_dma)
{
    TRACE("[SM] Creating Sampler %u with sampling period %u ms \n", id, period);

    if (is_dma)
    {
        return new DMASensorSampler(id, period);
    }
    else
    {
        return new SimpleSensorSampler(id, period);
    }
}