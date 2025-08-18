/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio, Fabrizio Monti
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

#include "SensorGroup.h"

namespace Boardcore
{

SensorManager::SensorManager(const SensorMap_t& sensorsMap)
{
    if (!init(sensorsMap, nullptr))
        LOG_ERR(logger, "Initialization failed");
}

SensorManager::SensorManager(const SensorMap_t& sensorsMap,
                             const SchedulerMap_t& schedulerMap)
{
    if (!init(sensorsMap, &schedulerMap))
        LOG_ERR(logger, "Initialization failed");
}

bool SensorManager::start(const SensorGroup::GroupId_t groupId)
{
    if (groups.count(groupId) != 0)
    {
        groups.at(groupId)->start();

        // TODO: should still return the init result, even if
        // it could be false because of another group?
        // Should I return the result of SensorGroup::start()?
        return initResult;
    }
    else
    {
        LOG_ERR(logger,
                "Can't start the scheduler of group {}, it does not exist",
                groupId);
        return false;
    }
}

void SensorManager::startAll()
{
    for (auto it : groups)
        it.second->start();
}

void SensorManager::stop(const SensorGroup::GroupId_t groupId)
{
    if (groups.count(groupId) != 0)
    {
        groups.at(groupId)->stop();
    }
    else
    {
        LOG_ERR(logger,
                "Can't stop the scheduler of group {}, it does not exist",
                groupId);
    }
}

void SensorManager::stopAll()
{
    for (auto it : groups)
        it.second->stop();
}

void SensorManager::enableSensor(AbstractSensor* sensor)
{
    if (groupsMap.find(sensor) != groupsMap.end())
    {
        groupsMap[sensor]->enableSensor(sensor);
    }
    else
    {
        LOG_ERR(logger, "Can't enable sensor {}, it does not exist",
                static_cast<void*>(sensor));
    }
}

void SensorManager::disableSensor(AbstractSensor* sensor)
{
    if (groupsMap.find(sensor) != groupsMap.end())
    {
        groupsMap[sensor]->disableSensor(sensor);
    }
    else
    {
        LOG_ERR(logger, "Can't disable sensor {}, it does not exist",
                static_cast<void*>(sensor));
    }
}

void SensorManager::enableAllSensors()
{
    for (auto it : groups)
        it.second->enableAllSensors();
}

void SensorManager::enableAllSensors(const SensorGroup::GroupId_t groupId)
{
    if (groups.find(groupId) != groups.end())
    {
        groups.at(groupId)->enableAllSensors();
    }
    else
    {
        LOG_ERR(logger, "Can't enable sensors, group {} does not exist",
                groupId);
    }
}

void SensorManager::disableAllSensors()
{
    for (auto it : groups)
        it.second->disableAllSensors();
}

void SensorManager::disableAllSensors(const SensorGroup::GroupId_t groupId)
{
    if (groups.find(groupId) != groups.end())
    {
        groups.at(groupId)->disableAllSensors();
    }
    else
    {
        LOG_ERR(logger, "Can't disable sensors, group {} does not exist",
                groupId);
    }
}

bool SensorManager::areAllSensorsInitialized() { return initResult; }

const SensorInfo SensorManager::getSensorInfo(AbstractSensor* sensor)
{
    if (groupsMap.find(sensor) != groupsMap.end())
        return groupsMap[sensor]->getSensorInfo(sensor);

    LOG_ERR(logger, "Sensor {} not found, can't return SensorInfo",
            static_cast<void*>(sensor));

    return SensorInfo{};
}

const vector<TaskStatsResult> SensorManager::getSamplersStats(
    const SensorGroup::GroupId_t groupId)
{
    if (groups.count(groupId) != 0)
        return groups.at(groupId)->getSamplersStats();

    LOG_ERR(logger, "Group {} not found, can't return sampler stats", groupId);

    return {};
}

bool SensorManager::init(const SensorMap_t& sensorsMap,
                         const SchedulerMap_t* schedulerMap)
{
    for (auto it : sensorsMap)
    {
        AbstractSensor* sensor = it.first;
        SensorInfo sensorInfo  = it.second;

        LOG_DEBUG(logger, "Initializing sensor {}", sensorInfo.id);
        // Try to initialize the sensors
        if (!initSensor(sensor))
        {
            sensorInfo.isEnabled = false;

            initResult = false;

            LOG_ERR(
                logger,
                "Failed to initialize sensor {} -> Error: {} (period: {} ns)",
                sensorInfo.id.c_str(), sensor->getLastError(),
                sensorInfo.period.count());
        }
        else
        {
            sensorInfo.isInitialized = true;
        }

        // Add sensor even if not initialized correctly, its isInitialized info
        // field will be false
        LOG_DEBUG(logger, "Adding {} -> period: {} ns, enabled = {}",
                  sensorInfo.id.c_str(), sensorInfo.period.count(),
                  sensorInfo.isEnabled);

        // Verify if a group for this sensor already exists
        if (groups.count(sensorInfo.groupID) == 0)
        {
            // Create the group
            if (schedulerMap == nullptr)
            {
                groups.emplace(
                    sensorInfo.groupID,
                    std::make_shared<SensorGroup>(sensorInfo.groupID));
            }
            else
            {
                // TODO: add check that the group exists inside schedulerMap?
                groups.emplace(sensorInfo.groupID,
                               std::make_shared<SensorGroup>(
                                   sensorInfo.groupID,
                                   schedulerMap->at(sensorInfo.groupID)));
            }
        }

        auto ptrGroup = groups.at(sensorInfo.groupID);

        ptrGroup->addSensor(sensor, sensorInfo);

        groupsMap.emplace(sensor, ptrGroup);
    }

    for (auto it : groups)
        it.second->initScheduler();

    return initResult;
}

bool SensorManager::initSensor(AbstractSensor* sensor)
{
    return sensor->init() && sensor->selfTest();
}

}  // namespace Boardcore
