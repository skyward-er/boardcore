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

#include <scheduler/TaskScheduler.h>
#include <sensors/SensorSampler.h>

using std::vector;

/**
 * The SensorManager class manages all the sensors connected to the Board.
 *
 * Sensors are grouped by "type" (Simple or DMA) and "sample frequency" and
 * grouped in various SensorSampler objects. These objects are then added to the
 * scheduler that manages the timings for the sampling.
 * After a SensorSampler has finished sampling its sensors, it will call a
 * callback, where these samples can be processed and dispatched.
 */
class SensorManager
{
public:
    
    typedef std::function<void()> function_t;

    /**
     * @brief Structure used to keep track of already existing
     *        SimplerSensorSampler, their frequency and the function 
     *        to be passed to the scheduler.
     */
    struct Sampler_t {
        SensorSampler* sampler;   // the sensor sampler
        function_t sampler_update_function;  // function periodically called by the scheduler
    };

    SensorManager();

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
     * @brief Add a sensor to be sampled with a SensorSampler 
     *        and the corresponding callback to be called at the given frequency.
     * 
     * @param sensor        the sensor to be added
     * @param freq          the frequency at which the sensor must be sampled
     * @param callback      the function to be called after the sensor has been sampled
     * @param sampler_type  the type of the sampler, SIMPLE_SAMPLER or DMA_SAMPLER
     * @return boolen value indicating whether the operation complete successfully or not
     */
    bool addSensor(Sensor* sensor, uint32_t freq, 
                                        function_t callback, 
                                        SamplerType sampler_type=SIMPLE_SAMPLER);

    /**
     * @brief Add a callback to be called at the given frequency.
     * 
     * @param freq      the frequency at which the function must be called
     * @param callback  the function to be called periodically
     * @param id        the identifier for the task in the scheduler
     */
    void addCallback(uint32_t freq, function_t callback, uint8_t id);

    /**
     * @return refernece to the samplers vector
     */
    vector<Sampler_t>& getSamplers();

    /**
     * @return  reference to the sensors sampling task scheduler
     */
    TaskScheduler& getScheduler();

    //SensorManagerStatus getStatus() { return status; }

private:
    /**
     * @brief Initialize a sensor and run the self-test.
     * 
     * @param sensor  the sensor to be initialized
     */
    bool initSensor(Sensor* sensor);

    /**
     * @brief Add all the SensorSamplers to the scheduler and begins sampling.
     */
    void initScheduler();


    TaskScheduler scheduler;

    // vectors containing samplers
    vector<Sampler_t> samplers;

    //SensorManagerStatus status;
    //SensorStatus sensor_status;

    uint32_t current_id = 0;  // incrementally assign IDs to scheduler tasks
};
