/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

#include "SensorGroup.h"

namespace Boardcore
{

SensorGroup::SensorGroup(uint8_t groupId, TaskScheduler* sched)
    : groupID(groupId), scheduler(sched),
      logger(Logging::getLogger("sensorgroup-" + std::to_string(groupId)))
{
    if (scheduler == nullptr)
    {
        customScheduler = true;
        scheduler       = new TaskScheduler();
        LOG_DEBUG(logger, "Allocated new task scheduler");
    }
    else
    {
        LOG_DEBUG(logger, "Task scheduler taken from the user");
        customScheduler = false;
    }

    uint8_t currentSamplerId = getFirstTaskID();
    if (currentSamplerId != 0)
    {
        LOG_DEBUG(logger, "Task scheduler not empty: starting from task ID {}",
                  currentSamplerId);
    }
}

SensorGroup::~SensorGroup()
{
    if (customScheduler)
        delete scheduler;
}

void SensorGroup::addSensor(AbstractSensor* sensor,
                            const SensorConfig& sensorConfig, const bool isInit)
{
    const uint8_t currentSamplerId = getFirstTaskID();

    // Check if a sampler with the same sampling period exists
    bool found = false;
    for (auto& sampler : samplers)
    {
        if (sensorConfig.period == sampler->getSamplingPeriod())
        {
            sampler->addSensor(sensor, sensorConfig, isInit);
            samplersMap[sensor] = sampler;
            found               = true;
        }
    }

    if (!found)
    {
        // A sampler with the required period does not exist yet
        auto newSampler = createSampler(currentSamplerId, sensorConfig.period);

        newSampler->addSensor(sensor, sensorConfig, isInit);

        samplers.push_back(newSampler);
        samplersMap[sensor] = newSampler;

        if (currentSamplerId == MAX_TASK_ID)
        {
            LOG_WARN(logger,
                     "Max task ID (255) reached in task scheduler, IDs "
                     "will start again from 0");
        }
    }

    LOG_DEBUG(logger, "Adding sensor {} to the group", sensorConfig.id);
}

void SensorGroup::enableSensor(AbstractSensor* sensor)
{
    if (samplersMap.find(sensor) != samplersMap.end())
    {
        samplersMap[sensor]->toggleSensor(sensor, true);
    }
    else
    {
        LOG_ERR(logger, "Can't enable sensor {}, it does not exist",
                static_cast<void*>(sensor));
    }
}

void SensorGroup::enableAllSensors()
{
    for (auto& it : samplers)
        it->enableAllSensors();
}

void SensorGroup::disableSensor(AbstractSensor* sensor)
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

void SensorGroup::disableAllSensors()
{
    for (auto& it : samplers)
        it->disableAllSensors();
}

void SensorGroup::start()
{
    if (customScheduler)
        scheduler->start();
}

void SensorGroup::stop() { scheduler->stop(); }

void SensorGroup::initScheduler()
{
    // Sort the vector to have lower period samplers (higher frequency) inserted
    // before higher period ones into the TaskScheduler
    std::stable_sort(samplers.begin(), samplers.end(),
                     SensorSampler::compareByPeriod);

    // Add all the samplers to the scheduler
    for (auto& sampler : samplers)
    {
        function_t samplerUpdateFunction([=]()
                                         { sampler->sampleAndCallback(); });

        scheduler->addTask(samplerUpdateFunction, sampler->getSamplingPeriod(),
                           TaskScheduler::Policy::RECOVER);
    }
}

const SensorInfo SensorGroup::getSensorInfo(AbstractSensor* sensor)
{
    if (samplersMap.count(sensor) != 0)
    {
        return samplersMap.at(sensor)->getSensorInfo(sensor);
    }
    else
    {
        LOG_ERR(logger, "Can't get sensor info, sensor {} does not exist",
                static_cast<void*>(sensor));
        return {};
    }
}

const vector<TaskStatsResult> SensorGroup::getSamplersStats()
{
    return scheduler->getTaskStats();
}

uint8_t SensorGroup::getFirstTaskID()
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

std::shared_ptr<SensorSampler> SensorGroup::createSampler(
    uint8_t id, std::chrono::nanoseconds period)
{
    LOG_DEBUG(logger, "Creating Sampler {} with sampling period {} ns", id,
              period.count());

    return std::make_shared<SimpleSensorSampler>(id, period);
}

}  // namespace Boardcore
