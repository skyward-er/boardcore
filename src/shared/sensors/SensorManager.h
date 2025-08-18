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

#include "SensorGroup.h"
#include "SensorInfo.h"
#include "SensorSampler.h"

namespace Boardcore
{

/**
 * @brief The SensorManager handles sensors initialization and sampling.
 *
 * Sensors are grouped by sampling period in various SensorSampler objects.
 * These samplers are then added to the scheduler ordered by sampling period.
 * The scheduler then manages the samplers by calling their sampleAndCallback
 * method periodically.
 *
 * TODO: The SensorManager should be able to reinitialized sensors that have
 * failed previous initializations.
 */
class SensorManager
{
public:
    // using function_t  = std::function<void()>;
    using SensorMap_t    = std::map<AbstractSensor*, SensorInfo>;
    using SchedulerMap_t = std::map<SensorGroup::GroupId_t, TaskScheduler*>;

    explicit SensorManager(const SensorMap_t& sensorsMap);

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

    void enableAllSensors();
    void enableAllSensors(const SensorGroup::GroupId_t groupId);

    void disableAllSensors();
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
     * @brief Initializes samplers vector and sensorsMap with the given sensors
     * map, giving incremental IDs to SensorSampler objects.
     *
     * In case a TaskScheduler was passed in the constructor, the SensorManager
     * will assign to SensorSamplers incremental IDs starting from the maximum
     * among the tasks already existing in the TaskScheduler.
     *
     * @param sensorsMap Map containing sensors and their respective information
     * for the sampling.
     */
    bool init(const SensorMap_t& sensorsMap,
              const SchedulerMap_t* schedulerMap);

    /**
     * @brief Initialize a sensor and run its self-test.
     *
     * @param sensor  the sensor to be initialized
     */
    bool initSensor(AbstractSensor* sensor);

    std::map<AbstractSensor*, std::shared_ptr<SensorGroup>> groupsMap;

    // TODO: vector instead of map?
    std::map<SensorGroup::GroupId_t, std::shared_ptr<SensorGroup>> groups;

    bool initResult = true;  ///< true if sensors are initialized correctly.

    PrintLogger logger = Logging::getLogger("sensormanager");
};

}  // namespace Boardcore
