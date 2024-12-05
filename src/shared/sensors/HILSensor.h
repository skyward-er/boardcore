/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Emilio Corigliano
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

#include <miosix.h>

#include <type_traits>

#include "HILSimulatorData.h"
#include "Sensor.h"

namespace Boardcore
{

/**
 * @brief Class that wraps a real sensor to perform HIL simulations.
 *
 * This class extends a real sensor class T. For the real sensor construction is
 * used the move constructor of an already created T sensor. The sampleImpl
 * method is overridden in order to expose the sample returned by the updateData
 * function passed in the constructor.
 * In this way, the corresponding HILSensor behaves exactly like a sensor T, but
 * returns the data given by the simulator.
 *
 * The real hardware sensor can be used by creating an instance of this class
 * with the `enableHw` constructor parameter set to `true`. In that case,
 * when the `sample` method is called, the real sensor is sampled and then
 * the data from the simulator is mapped into the SensorData struct.
 */

template <class T>
class HILSensor : public T
{
public:
    // A requirement is that the class T must be move constructible
    static_assert(std::is_move_constructible<T>::value,
                  "T must be move constructible!");
    // A requirement is that the class T must be a Sensor
    static_assert(std::is_base_of<Sensor<typename T::DataType>, T>::value,
                  "T must inherit from Sensor<T::DataType>");

    using UpdateFn = std::function<typename T::DataType(void)>;

    /**
     * @brief Constructor of the HILSensor which decorates the passed sensor
     * with the sampling of the simulator data.
     *
     * @param sensor The sensor to be wrapped.
     * @param enableHw Flag to enable the sampling of the real hardware.
     * @param updateData Function that maps the data from the simulator data to
     * the sensor current sample.
     */
    HILSensor(T&& sensor, bool enableHw, UpdateFn updateData)
        : T{std::move(sensor)}, enableHw{enableHw},
          updateData{std::move(updateData)}
    {
    }

    bool init() override { return (enableHw ? T::init() : true); }

    bool selfTest() override { return (enableHw ? T::selfTest() : true); }

    /**
     * @brief Overridden sampleImpl method so that the data could be actually
     * taken from the simulator.
     *
     * If the hardware is enabled, it also samples the real sensor.
     *
     * @return The data struct with the current simulator sample.
     */
    typename T::DataType sampleImpl() override
    {
        if (enableHw)
        {
            auto realSample = T::sampleImpl();
            {
                miosix::Lock<miosix::FastMutex> l(mutexRealData);
                lastRealSample = realSample;
            }
        }

        return updateData();
    }

    /**
     * @brief Method analogous to getLastSample but returns the real sample
     * taken from the hardware sensor.
     *
     * @return The data struct with the current sensor sample.
     */
    typename T::DataType getRealLastSample()
    {
        miosix::Lock<miosix::FastMutex> l(mutexRealData);
        return lastRealSample;
    }

protected:
    typename T::DataType lastRealSample;  // Last sample of the real sensor
    bool enableHw;                        // Flag to enable hardware sampling
    UpdateFn updateData;  // Function to get the lastSample from simulatorData

    // Thread safe mutex to synchronize writes and reads on lastRealSample
    miosix::FastMutex mutexRealData;
};

/**
 * @brief Static method that hillificates sensors, turning a real sensor in
 * a HILSensor.
 *
 * Creates a HILSensor of the sensor type T. The passed sensor must be an
 * already instantiated sensor.
 */
template <typename T>
static void hillificator(std::unique_ptr<T>& sensor, bool enableHw,
                         typename HILSensor<T>::UpdateFn updateData)
{
    if (sensor)
    {
        sensor = std::make_unique<Boardcore::HILSensor<T>>(
            std::move(*sensor), enableHw, std::move(updateData));
    }
}

}  // namespace Boardcore
