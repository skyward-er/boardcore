/* Copyright (c) 2017-2020 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Luca Conterio
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

#include <Common.h>
#include <diagnostic/PrintLogger.h>

#include "Sensor.h"
#include "SensorInfo.h"

namespace Boardcore
{

/**
 * @brief Virtual sensor sampler class.
 *
 * When requested, a SensorSampler samples all the enabled sensors it manages.
 * After sampling a sensor, the SensorSampler calls the callback associated
 * with that sensor, where these samples can be processed.
 */
class SensorSampler
{
public:
    /**
     * @param id      sampler identifier
     * @param period  period at which the sampler performs sensors update
     * @param is_dma  indicate if the sampler manages DMA sensors or not
     */
    SensorSampler(uint8_t id, uint32_t period, bool is_dma);

    virtual ~SensorSampler();

    /**
     * @brief Add a sensor to the sensors map.
     *
     * @param sensor  the sensor to be added
     */
    virtual void addSensor(AbstractSensor* sensor, SensorInfo sensor_info) = 0;

    /**
     * @brief Enabled or disable a sensor.
     *
     * @param sensor    the sensor to be toggled (enabled/disabled)
     * @param is_en     bool value to be set in the sensor info to indicate if
     *                  the sensor will be enabled or disabled
     */
    void toggleSensor(AbstractSensor* sensor, bool is_en);

    /**
     * @brief Enable sampling for all the sensors.
     */
    void enableAllSensors();

    /**
     * @brief Disable sampling for all the sensors.
     */
    void disableAllSensors();

    /**
     * @brief For each sensor, sample it and call the corresponding callback.
     */
    void sampleAndCallback();

    /**
     * @return  the sampler's type
     */
    bool isDMA();

    /**
     * @return  the sampler's ID
     */
    uint8_t getID();

    /**
     * @return  the sampler's activation period
     */
    uint32_t getSamplingPeriod();

    /**
     * @return  the number of sensors assigned to this sampler
     */
    unsigned int getNumSensors();

    /**
     * @return  the information related to the given sensor
     */
    const SensorInfo getSensorInfo(AbstractSensor* sensor);

private:
    /**
     * @brief Perform the update of all the sensors in the sampler.
     */
    virtual void sampleSensor(AbstractSensor* s) = 0;

    uint8_t id;      /**< sampler id used in the task scheduler */
    uint32_t period; /**< sampler update/activation period */
    bool is_dma;     /**< the sampler's type (if it uses DMA or not) */

protected:
    std::vector<std::pair<AbstractSensor*, SensorInfo>> sensors;

    PrintLogger logger = Logging::getLogger("sensorsampler");
};

/**
 * @brief Sampler for simple sensors, those that are simply
 *        sampled by calling the sample() method.
 */
class SimpleSensorSampler : public virtual SensorSampler
{
public:
    SimpleSensorSampler(uint8_t id, uint32_t period);

    ~SimpleSensorSampler();

    void addSensor(AbstractSensor* sensor, SensorInfo sensor_info) override;

    void sampleSensor(AbstractSensor* s) override;

private:
    SimpleSensorSampler(const SimpleSensorSampler&) = delete;
};

/**
 * @brief Sampler for sensors that use DMA for updates.
 */
class DMASensorSampler : public virtual SensorSampler
{
public:
    DMASensorSampler(uint8_t id, uint32_t period);

    ~DMASensorSampler();

    void addSensor(AbstractSensor* sensor, SensorInfo sensor_info) override;

    void sampleSensor(AbstractSensor* s) override;

private:
    DMASensorSampler(const DMASensorSampler&) = delete;
};

}  // namespace Boardcore
