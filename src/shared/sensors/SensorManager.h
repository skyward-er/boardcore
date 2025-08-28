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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>

#include <map>
#include <memory>

#include "SensorConfig.h"
#include "SensorGroup.h"
#include "SensorInfo.h"
#include "SensorSampler.h"

namespace Boardcore
{

/**
 * @brief The SensorManager handles sensors and their initialization.
 * They are initialized and then grouped inside sensor groups, which
 * will take care of their sampling.
 *
 * TODO: The SensorManager should be able to reinitialized sensors that have
 * failed previous initializations.
 */
class SensorManager
{
public:
    using SensorMap_t    = std::map<AbstractSensor*, SensorConfig>;
    using SchedulerMap_t = std::map<SensorGroup::GroupId_t, TaskScheduler*>;

    /**
     * @param sensorMap Map containing sensors and their respective information
     * for the sampling.
     */
    explicit SensorManager(const SensorMap_t& sensorsMap);

    /**
     * @param sensorMap Map containing sensors and their respective information
     * for the sampling.
     * @param schedulerMap Mapping of the schedulers for each SensorGroup. If
     * no scheduler is specified for a specific group, a new one is created.
     */
    SensorManager(const SensorMap_t& sensorsMap,
                  const SchedulerMap_t& schedulerMap);

    /**
     * @brief Starts the task scheduler of the group.
     */
    bool start(const SensorGroup::GroupId_t groupId);

    /**
     * @brief Starts the task scheduler for every group.
     */
    void startAll();

    /**
     * @brief Stops the task scheduler of the group.
     */
    void stop(const SensorGroup::GroupId_t groupId);

    /**
     * @brief Stops the task scheduler for every group.
     */
    void stopAll();

    /**
     * @brief Enable sampling for the specified sensor.
     *
     * @param sensor The sensor to be enabled.
     */
    void enableSensor(AbstractSensor* sensor);

    /**
     * @brief Disable sampling for the specified sensor.
     *
     * @param sensor The sensor to be disabled.
     */
    void disableSensor(AbstractSensor* sensor);

    /**
     * @brief Enable all the sensors for all the sensor groups.
     */
    void enableAllSensors();

    /**
     * @brief Enable all the sensors for the specific group.
     */
    void enableAllSensors(const SensorGroup::GroupId_t groupId);

    /**
     * @brief Disable all the sensors for all the sensor groups.
     */
    void disableAllSensors();

    /**
     * @brief Disable all the sensors for the specific group.
     */
    void disableAllSensors(const SensorGroup::GroupId_t groupId);

    /**
     * @brief Checks whether all the sensors have been initialized correctly.
     */
    bool areAllSensorsInitialized();

    const SensorInfo getSensorInfo(AbstractSensor* sensor);

    /**
     * @return Vector of statistics, one for each sampler, taken from the
     * scheduler of the group.
     */
    const vector<TaskStatsResult> getSamplersStats(
        const SensorGroup::GroupId_t groupId);

private:
    SensorManager(const SensorManager&)            = delete;
    SensorManager& operator=(const SensorManager&) = delete;

    /**
     * @brief Initializes sensors and groups with the given sensors map and
     * schedulers map.
     *
     * @param sensorsMap Map containing sensors and their respective information
     * for the sampling.
     * @param schedulerMap A pointer (can be nullptr) to the map that contains
     * the scheduler that each sensor group should use. If nullptr (or no
     * scheduler is provided for a certain group) the sensor group will allocate
     * a new scheduler.
     */
    bool init(const SensorMap_t& sensorsMap,
              const SchedulerMap_t* schedulerMap);

    /**
     * @brief Initialize a sensor and run its self-test.
     *
     * @param sensor The sensor to be initialized.
     */
    bool initSensor(AbstractSensor* sensor);

    /**
     * @brief Mapping between the sensor and the sensor group is
     * associated to.
     */
    std::map<AbstractSensor*, std::shared_ptr<SensorGroup>> groupsMap;

    /**
     * @brief Mapping between the sensor groups and their id.
     */
    std::map<SensorGroup::GroupId_t, std::shared_ptr<SensorGroup>> groups;

    bool initResult = true;  ///< true if sensors are initialized correctly.

    PrintLogger logger = Logging::getLogger("sensormanager");
};

}  // namespace Boardcore
