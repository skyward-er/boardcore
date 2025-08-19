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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>

#include <map>
#include <memory>

#include "SensorInfo.h"
#include "SensorSampler.h"

namespace Boardcore
{

class SensorGroup
{
public:
    using function_t = std::function<void()>;
    using GroupId_t  = uint8_t;

    SensorGroup(GroupId_t groupId, TaskScheduler* scheduler = nullptr);

    ~SensorGroup();

    /**
     * @brief Start the scheduler, only if it was not provided from the outside.
     */
    void start();

    /**
     * @brief Stop the scheduler.
     */
    void stop();

    /**
     * @brief Initialize scheduler by adding all the SensorSamplers tasks.
     */
    void initScheduler();

    /**
     * @brief Add a sensor to the sensor group.
     */
    void addSensor(AbstractSensor* sensor, const SensorInfo& sensorInfo);

    void enableSensor(AbstractSensor* sensor);

    /**
     * @brief Enable all the sensors in the group.
     */
    void enableAllSensors();

    void disableSensor(AbstractSensor* sensor);

    /**
     * @brief Disable all the sensors in the group.
     */
    void disableAllSensors();

    const SensorInfo getSensorInfo(AbstractSensor* sensor);

    const vector<TaskStatsResult> getSamplersStats();

    /**
     * @return The group id.
     */
    GroupId_t getId() { return groupID; }

private:
    /**
     * @brief Avoid creating duplicate IDs for tasks in case the scheduler is
     * received from outside.
     *
     * @return Maximum ID among those assigned to tasks in the scheduler.
     */
    uint8_t getFirstTaskID();

    /**
     * @brief Create a sampler object with the specified sampling period.
     *
     * @param id New sampler's identifier.
     * @param period Sampling period of the new sampler.
     *
     * @return Pointer to the newly created sampler.
     */
    std::shared_ptr<SensorSampler> createSampler(
        uint8_t id, std::chrono::nanoseconds period);

    const GroupId_t groupID;

    static constexpr uint8_t MAX_TASK_ID =
        255;  ///< Max id for tasks in the scheduler.

    /**
     * @brief Needed to update the samplers at the correct period.
     * TODO: can it be substituted with smart pointers?
     */
    TaskScheduler* scheduler;

    /**
     * @brief True if the scheduler was not provided by the user (we created
     * it).
     */
    bool customScheduler;

    std::vector<std::shared_ptr<SensorSampler>>
        samplers;  ///< Vector of all the samplers (unique).

    std::map<AbstractSensor*, std::shared_ptr<SensorSampler>>
        samplersMap;  ///< Map each sensor to the corresponding sampler.

    std::map<AbstractSensor*, SensorInfo>
        sensorMap;  ///< Map each sensor to the corresponding SensorInfo.

    PrintLogger logger;
};

}  // namespace Boardcore
