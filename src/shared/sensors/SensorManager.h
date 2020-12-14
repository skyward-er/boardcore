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

    SensorManager(const SensorMap_t& sensors_map);

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
     * @brief Initializes samplers vector and sensors_map with the given sensors
     * map.
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
     * @brief Create a sampler object according to the fact that it needs to use
     * DMA or not.
     *
     * @param is_dma    indicate if the sampler manages sensors that use DMA
     * @param freq      sampling frequency of the new sampler
     * @param id        new sampler's identifier
     *
     * @return pointer to the newly created sampler
     */
    SensorSampler* createSampler(uint32_t id, uint32_t freq, bool is_dma);

    TaskScheduler scheduler; /**< scheduler to update the samplers at the
                                correct frequency */

    vector<SensorSampler*> samplers; /**< vector of all the samplers (unique) */

    map<AbstractSensor*, SensorSampler*>
        samplers_map; /**< map each sensor to the corresponding sampler */
};
