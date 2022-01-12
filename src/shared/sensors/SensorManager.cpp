/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include "SensorManager.h"

#include <utils/Debug.h>

namespace Boardcore
{

SensorManager::SensorManager(const SensorMap_t& sensors_map)
    : SensorManager(new TaskScheduler(), sensors_map)
{
}

SensorManager::SensorManager(TaskScheduler* scheduler,
                             const SensorMap_t& sensors_map)
    : scheduler(scheduler)
{
    if (!init(sensors_map))
    {
        LOG_ERR(logger, "Initialization failed");
    }
}

SensorManager::~SensorManager()
{
    for (auto s : samplers)
    {
        delete s;
    }
}

bool SensorManager::start() { return scheduler->start() && sensorsInitResult; }

void SensorManager::stop() { scheduler->stop(); }

void SensorManager::enableSensor(AbstractSensor* sensor)
{
    if (samplersMap.find(sensor) != samplersMap.end())
    {
        samplersMap[sensor]->toggleSensor(sensor, true);
    }
    else
    {
        LOG_ERR(logger, "Can't enable sensor {} it does not exist",
                static_cast<void*>(sensor));
    }
}

void SensorManager::disableSensor(AbstractSensor* sensor)
{
    if (samplersMap.find(sensor) != samplersMap.end())
    {
        samplersMap[sensor]->toggleSensor(sensor, false);
    }
    else
    {
        LOG_ERR(logger, "Can't disable sensor {}, it does not exist",
                static_cast<void*>(sensor));
    }
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

const SensorInfo SensorManager::getSensorInfo(AbstractSensor* sensor)
{
    if (samplersMap.find(sensor) != samplersMap.end())
    {
        return samplersMap[sensor]->getSensorInfo(sensor);
    }

    LOG_ERR(logger, "Sensor {} not found, can't return SensorInfo",
            static_cast<void*>(sensor));

    return SensorInfo{};
}

const vector<TaskStatResult> SensorManager::getSamplersStats()
{
    return scheduler->getTaskStats();
}

bool SensorManager::init(const SensorMap_t& sensors_map)
{
    uint8_t current_sampler_id = getFirstTaskID();

    if (current_sampler_id != 0)
    {
        LOG_INFO(logger, "Task scheduler not empty: starting from task ID {}",
                 current_sampler_id);
    }

    for (auto it = sensors_map.begin(); it != sensors_map.end(); it++)
    {
        AbstractSensor* sensor = it->first;
        SensorInfo sensor_info = it->second;

        // avoid adding sensors that fail to be initalized
        if (!initSensor(sensor))
        {
            sensor_info.isEnabled = false;  // disable the failing sensor

            sensorsInitResult = false;

            LOG_ERR(
                logger,
                "Failed to initialize sensor {} -> Error: {} (period: {} ms)",
                sensor_info.id.c_str(), sensor->getLastError(),
                sensor_info.period);
        }
        else
        {
            sensor_info.isInitialized = true;
        }

        // add sensor even if not initialized correctly, its is_initialized info
        // field will be false
        LOG_DEBUG(logger, "Adding {} -> period: {} ms, enabled = {}",
                  sensor_info.id.c_str(), sensor_info.period,
                  sensor_info.isEnabled);

        // check if a sampler with the same sampling period and the same
        // type exists
        bool found = false;
        for (auto s : samplers)
        {
            if (sensor_info.period == s->getSamplingPeriod() &&
                sensor_info.isDma == s->isDMA())
            {
                s->addSensor(sensor, sensor_info);
                samplersMap[sensor] = s;
                found               = true;
            }
        }

        if (!found)
        {
            // a sampler with the required period does not exist yet
            SensorSampler* new_sampler = createSampler(
                current_sampler_id, sensor_info.period, sensor_info.isDma);

            new_sampler->addSensor(sensor, sensor_info);

            samplers.push_back(new_sampler);
            samplersMap[sensor] = new_sampler;

            if (current_sampler_id == MAX_TASK_ID)
            {
                LOG_WARN(logger,
                         "Max task ID (255) reached in task scheduler, IDs "
                         "will start again from 0");
            }

            current_sampler_id++;
        }
    }

    initScheduler();

    return sensorsInitResult;
}

bool SensorManager::initSensor(AbstractSensor* sensor)
{
    return sensor->init() && sensor->selfTest();
}

void SensorManager::initScheduler()
{
    // sort the vector to have lower period samplers
    // (higher frequency) inserted before
    // higher period ones into the TaskScheduler
    std::sort(samplers.begin(), samplers.end(),
              [](auto& left, auto& right) {
                  return left->getSamplingPeriod() < right->getSamplingPeriod();
              });

    uint64_t start_time = miosix::getTick() + 10;

    // add all the samplers to the scheduler
    for (auto& s : samplers)
    {
        function_t sampler_update_function =
            std::bind(&SensorSampler::sampleAndCallback, s);

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

    auto max =
        std::max_element(tasks_stats.begin(), tasks_stats.end(),
                         [](const TaskStatResult& t1, const TaskStatResult& t2)
                         { return t1.id < t2.id; });

    return max->id + 1;
}

SensorSampler* SensorManager::createSampler(uint8_t id, uint32_t period,
                                            bool isDma)
{
    LOG_DEBUG(logger, "Creating Sampler {} with sampling period {} ms", id,
              period);

    if (isDma)
    {
        return new DMASensorSampler(id, period);
    }
    else
    {
        return new SimpleSensorSampler(id, period);
    }
}

}  // namespace Boardcore
