/* Copyright (c) 2017-2020 Skyward Experimental Rocketry
 * Author: Alain Carlucci, Luca Conterio
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
#ifndef SENSOR_SAMPLING_H
#define SENSOR_SAMPLING_H

#include <Common.h>
#include "Sensor.h"

using namespace std;

/**
 * Enum to distinguish the sensor sampler type,
 * if it a simple one or if it uses DMA.
 */
enum SamplerType {
    SIMPLE_SAMPLER = 0,
    DMA_SAMPLER    = 1
};


/**
 * Virtual sensor sampler class.
 */
class SensorSampler 
{
    public: 

        typedef function<void()> function_t;

        /**
         * @brief Constructor.
         * 
         * @param type  sampler type, SIMPLER_SAMPLER or DMA_SAMPLER
         * @param freq  frequency at which the sampler performs sensors update
         */
        SensorSampler(SamplerType type, uint32_t freq, uint32_t id);

        /**
         * @brief Add a sensor to the sensors vector.
         * 
         * @param sensor  the sensor to be added
         */
        virtual void addSensor(Sensor* sensor, function_t callback) = 0;

        /**
         * @brief For each sensor, sample it and call the corresponding callback.
         */
        void sampleAndCallback();

        /**
         * @return  the sampler's type
         */
        SamplerType getType();

        /**
         * @return  the sampler's ID
         */
        uint32_t getId();

        /**
         * @return  the sampler's activation frequency
         */
        uint32_t getFrequency();

        /**
         * @return  the number of sensors assigned to this sampler
         */
        uint32_t getNumSensors();
    
    private:
        /**
         * @brief Perform the update of all the sensors in the sampler.
         */
        virtual void sampleSensor(Sensor* s) = 0;

        SamplerType type;  // the sampler's type
        uint32_t freq; // frequency at which the sampler performs sensors update
        uint32_t id;   // id used for scheduler task

    protected:
        map<Sensor*, function_t> sensors_map;  // for each sensor a callback

};


/**
 * @brief Sampler for simple sensors, those that are simply
 *        sampled by calling the onSimpleUpdate() method.
 */
class SimpleSensorSampler : public SensorSampler
    {
    public:
        SimpleSensorSampler(uint32_t freq, uint32_t id);

        ~SimpleSensorSampler();

        void addSensor(Sensor* sensor, function_t sensor_callback) override;

        void sampleSensor(Sensor* s) override;
};


/**
 * @brief Sampler for sensors that DMA for updates.
 */
class DMASensorSampler : public SensorSampler
{
    public:
        DMASensorSampler(uint32_t freq, uint32_t id);

        ~DMASensorSampler();

        void addSensor(Sensor* sensor, function_t sensor_callback) override;

        void sampleSensor(Sensor* s) override;
        
    private: 
        map<Sensor*, vector<SPIRequest>> requests_map;  // SPI requests needed to perform DMA updates
};

#endif /* ifndef SENSOR_SAMPLING_H */