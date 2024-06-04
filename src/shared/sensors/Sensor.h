/* Copyright (c) 2016-2020 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Luca Conterio, Davide Mor, Emilio Corigliano
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

#include <array>
#include <memory>
#include <type_traits>

#include "SensorData.h"

namespace Boardcore
{

/**
 * @brief Check that a given type has a method called `getData()` and that the
 * return type of this method is a subclass of the expected data type.
 */
template <class T, class ExpectedDataType>
struct checkIfProduces
    : std::is_base_of<ExpectedDataType,
                      typename std::remove_reference<
                          decltype(std::declval<T>().getLastSample())>::type>
{
};

/**
 * @brief Base abstract class for sensor drivers.
 */
class AbstractSensor
{
protected:
    SensorErrors lastError = SensorErrors::NO_ERRORS;

public:
    virtual ~AbstractSensor() {}

    /**
     * @brief Initialize the sensor.
     * @return boolean value indicating whether the operation succeded or not
     */
    virtual bool init() = 0;

    /**
     * @brief Check if the sensor is working.
     * @return boolean indicating whether the sensor is correctly working or not
     */
    virtual bool selfTest() = 0;

    /**
     * @brief Sample the sensor.
     */
    virtual void sample() = 0;

    /**
     * @brief Get last error for debugging purposes. Avoid silent fails.
     * @return the last error recorded by this sensor
     */
    SensorErrors getLastError() { return lastError; };
};

/**
 * @brief Base sensor class with has to be extended by any sensor driver.
 *
 * A sensor driver can define a custom data structure extending any
 * combination of base sensors data structures, defined in `SensorData.h`.
 */
template <typename Data>
class Sensor : public virtual AbstractSensor
{
public:
    using DataType = Data;

protected:
    DataType lastSample;

    /**
     * @brief Read a data sample from the sensor.
     *        In case of errors, the method should return the last
     *        available correct sample.
     * @return sensor data sample
     */
    virtual DataType sampleImpl() = 0;

    // Thread safe mutex to synchronize writes and reads
    miosix::FastMutex mutex;

public:
    Sensor() {}

    Sensor(Sensor&& other) : lastSample{std::move(other.lastSample)}, mutex{} {}

    virtual ~Sensor() {}

    void sample() override
    {
        // Sampling outside of the protected zone ensures that the sampling
        // action cannot cause locks or delays
        DataType d = sampleImpl();

        {
            miosix::Lock<miosix::FastMutex> l(mutex);
            lastSample = d;
        }
    }

    /**
     * @return last available sample from this sensor
     */
    virtual const DataType& getLastSample()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        return lastSample;
    }
};

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
        : T{std::move(sensor)}, enableHw{enableHw}, updateData{updateData}
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

/**
 * @brief Interface for sensor that implement a FIFO.
 */
template <typename Data, uint32_t FifoSize>
class SensorFIFO : public Sensor<Data>
{
protected:
    std::array<Data, FifoSize> lastFifo;
    uint16_t lastFifoLevel = 1;  ///< number of samples in lastFifo

    uint64_t lastInterruptTimestamp = 0;
    uint64_t interruptTimestampDelta =
        0;  ///< delta between previous interrupt timestamp and the last
            ///< received one

public:
    /**
     * @return last FIFO sampled from the sensor
     */
    const std::array<Data, FifoSize>& getLastFifo() { return lastFifo; }

    /**
     * @param i index of the requested item inside the FIFO
     *
     * @return the i-th element of the FIFO
     */
    const Data& getFifoElement(uint32_t i) const { return lastFifo[i]; }

    /**
     * @return number of elements in the last FIFO sampled from the sensor
     */
    uint16_t getLastFifoSize() const { return lastFifoLevel; }

    /**
     * @brief Called by the interrupt handling routine: provides the timestamp
     *        of the last interrupt (if FIFO is disabled) or the last watermark
     *        interrupt (if FIFO enabled)
     *
     * @param ts Timestamp of the lasts interrupt, in microseconds
     */
    inline virtual void IRQupdateTimestamp(uint64_t ts)
    {
        interruptTimestampDelta = ts - lastInterruptTimestamp;
        lastInterruptTimestamp  = ts;
    }
};

}  // namespace Boardcore
