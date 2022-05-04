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

SensorManager::SensorManager(const SensorMap_t& sensorsMap)
    : scheduler(new TaskScheduler()), customScheduler(true)
{
    if (!init(sensorsMap))
        LOG_ERR(logger, "Initialization failed");
}

SensorManager::SensorManager(const SensorMap_t& sensorsMap,
                             TaskScheduler* scheduler)
    : scheduler(scheduler), customScheduler(false)
{
    if (!init(sensorsMap))
        LOG_ERR(logger, "Initialization failed");
}

SensorManager::~SensorManager()
{
    for (auto sampler : samplers)
        delete sampler;
    if (customScheduler)
        delete scheduler;
}

bool SensorManager::start()
{
    if (customScheduler)
        scheduler->start();
    return initResult;
}

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
    for (auto sampler : samplers)
        sampler->enableAllSensors();
}

void SensorManager::disableAllSensors()
{
    for (auto sampler : samplers)
        sampler->disableAllSensors();
}

bool SensorManager::areAllSensorsInitialized() { return initResult; }

const SensorInfo SensorManager::getSensorInfo(AbstractSensor* sensor)
{
    if (samplersMap.find(sensor) != samplersMap.end())
        return samplersMap[sensor]->getSensorInfo(sensor);

    LOG_ERR(logger, "Sensor {} not found, can't return SensorInfo",
            static_cast<void*>(sensor));

    return SensorInfo{};
}

const vector<TaskStatsResult> SensorManager::getSamplersStats()
{
    return scheduler->getTaskStats();
}

bool SensorManager::init(const SensorMap_t& sensorsMap)
{
    uint8_t currentSamplerId = getFirstTaskID();

    if (currentSamplerId != 0)
    {
        LOG_INFO(logger, "Task scheduler not empty: starting from task ID {}",
                 currentSamplerId);
    }

    for (auto it : sensorsMap)
    {
        AbstractSensor* sensor = it.first;
        SensorInfo sensorInfo  = it.second;

        // Try to initialize the sensors
        if (!initSensor(sensor))
        {
            sensorInfo.isEnabled = false;

            initResult = false;

            LOG_ERR(
                logger,
                "Failed to initialize sensor {} -> Error: {} (period: {} ms)",
                sensorInfo.id.c_str(), sensor->getLastError(),
                sensorInfo.period);
        }
        else
        {
            sensorInfo.isInitialized = true;
        }

        // Add sensor even if not initialized correctly, its isInitialized info
        // field will be false
        LOG_DEBUG(logger, "Adding {} -> period: {} ms, enabled = {}",
                  sensorInfo.id.c_str(), sensorInfo.period,
                  sensorInfo.isEnabled);

        // Check if a sampler with the same sampling period exists
        bool found = false;
        for (auto sampler : samplers)
        {
            if (sensorInfo.period == sampler->getSamplingPeriod())
            {
                sampler->addSensor(sensor, sensorInfo);
                samplersMap[sensor] = sampler;
                found               = true;
            }
        }

        if (!found)
        {
            // A sampler with the required period does not exist yet
            SensorSampler* newSampler =
                createSampler(currentSamplerId, sensorInfo.period);

            newSampler->addSensor(sensor, sensorInfo);

            samplers.push_back(newSampler);
            samplersMap[sensor] = newSampler;

            if (currentSamplerId == MAX_TASK_ID)
                LOG_WARN(logger,
                         "Max task ID (255) reached in task scheduler, IDs "
                         "will start again from 0");

            currentSamplerId++;
        }
    }

    initScheduler();

    return initResult;
}

bool SensorManager::initSensor(AbstractSensor* sensor)
{
    return sensor->init() && sensor->selfTest();
}

void SensorManager::initScheduler()
{
    // Sort the vector to have lower period samplers (higher frequency) inserted
    // before higher period ones into the TaskScheduler
    std::stable_sort(samplers.begin(), samplers.end(),
                     SensorSampler::comparareByPeriod);

    // Add all the samplers to the scheduler
    for (auto& sampler : samplers)
    {
        function_t samplerUpdateFunction([=]()
                                         { sampler->sampleAndCallback(); });

        scheduler->addTask(samplerUpdateFunction, sampler->getSamplingPeriod(),
                           sampler->getID());
    }
}

uint8_t SensorManager::getFirstTaskID()
{
    std::vector<TaskStatsResult> tasksStats = scheduler->getTaskStats();

    if (tasksStats.empty())
        return 0;

    auto max = std::max_element(
        tasksStats.begin(), tasksStats.end(),
        [](const TaskStatsResult& t1, const TaskStatsResult& t2)
        { return t1.id < t2.id; });

    return max->id + 1;
}

SensorSampler* SensorManager::createSampler(uint8_t id, uint32_t period)
{
    LOG_DEBUG(logger, "Creating Sampler {} with sampling period {} ms", id,
              period);

    return new SimpleSensorSampler(id, period);
}

}  // namespace Boardcore
