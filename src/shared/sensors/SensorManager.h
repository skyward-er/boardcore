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

#pragma once

#include "SensorInfo.h"
#include "SensorSampler.h"
#include "scheduler/TaskScheduler.h"

using namespace std;

/**
 * @brief The SensorManager class manages all the sensors connected to the
 * Board.
 *
 * Sensors are grouped in various SensorSampler objects by "type" (if they use
 * DMA or not) and "sampling frequency". These SensorSampler objects are then
 * added to the scheduler that manages the timings for the sampling.
 */
class SensorManager
{
public:
    using function_t  = function<void()>;
    using SensorMap_t = map<AbstractSensor*, SensorInfo>;

    /**
     * @brief Constructor.
     *
     * @param sensors_map map containing references to the sensors as keys,
     *                    and objects of type SensorInfo as values.
     */
    SensorManager(const SensorMap_t& sensors_map);

    /**
     * @brief Constructor taking an external TaskScheduler object.
     *
     * @param sensors_map map containing references to the sensors as keys,
     *                    and objects of type SensorInfo as values.
     */
    SensorManager(TaskScheduler* scheduler, const SensorMap_t& sensors_map);

    /**
     * @brief Destructor.
     *
     * Deallocates samplers (through the samplers vector).
     */
    ~SensorManager();

    /**
     * @brief Start the sensor manager.
     */
    bool start();

    /**
     * @brief Stop the sensor manager.
     */
    void stop();

    /**
     * @brief Enable sampling for the passed sensor.
     *
     * @param sensor    the sensor to be enabled
     */
    void enableSensor(AbstractSensor* sensor);

    /**
     * @brief Disable sampling for the passed sensor.
     *
     * @param sensor    the sensor to be disabled
     */
    void disableSensor(AbstractSensor* sensor);

    /**
     * @brief Enable sampling for all the sensors.
     */
    void enableAllSensors();

    /**
     * @brief Disable sampling for all the sensors.
     */
    void disableAllSensors();

    /**
     * @return the information related to the given sensor
     */
    const SensorInfo& getSensorInfo(AbstractSensor* sensor);

    /**
     * @return vector of statistics, one for each sampler, taken from the
     * scheduler
     */
    const vector<TaskStatResult> getSamplersStats();

private:
    /**
     * @brief Copy constructor. Deleted.
     */
    SensorManager(const SensorManager&) = delete;

    /**
     * @brief Initializes samplers vector and sensors_map with the given sensors
     *        map, giving incremental IDs to SensorSampler objects.
     *        In case a TaskScheduler was passed in the costructor,
     *        the SensorManager will assign to SensorSamplers incremental IDs
     *        starting from the maximum among the tasks already existing
     *        in the TaskScheduler.
     *
     * @param sensors_map    map containing sensors and their respective
     * information for the sampling
     */
    bool init(const SensorMap_t& sensors_map);

    /**
     * @brief Initialize a sensor and run its self-test.
     *
     * @param sensor  the sensor to be initialized
     */
    bool initSensor(AbstractSensor* sensor);

    /**
     * @brief Initialize scheduler by adding all the SensorSamplers tasks.
     */
    void initScheduler();

    /**
     * @brief Avoid creating duplicate IDs for tasks in case the scheduler
     *        is received from outside.
     *
     * @return maximum ID among those assigned to tasks in the scheduler.
     */
    uint8_t getFirstTaskID();

    /**
     * @brief Create a sampler object according to the fact that it needs to use
     * DMA or not.
     *
     * @param is_dma    indicate if the sampler manages sensors that use DMA
     * @param freq      sampling frequency of the new sampler
     * @param id        new sampler's identifier
     *
     * @return pointer to the newly created sampler
     */
    SensorSampler* createSampler(uint8_t id, uint32_t freq, bool is_dma);

    const uint8_t MAX_TASK_ID = 255; /**< max id for tasks in the scheduler */

    TaskScheduler* scheduler; /**< scheduler to update the samplers at the
                                correct frequency */

    vector<SensorSampler*> samplers; /**< vector of all the samplers (unique) */

    map<AbstractSensor*, SensorSampler*>
        samplers_map; /**< map each sensor to the corresponding sampler */
};
