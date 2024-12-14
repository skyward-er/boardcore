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
    virtual Data getLastSample()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        return lastSample;
    }
};

/**
 * @brief Interface for sensor that implement a FIFO.
 *
 * @note This class is move constructible, but the move operation is not thread
 * safe. The moved-to object's mutex is initialized to the unlocked state, no
 * matter the moved-from object's mutex state. The move constructor is only
 * needed for HIL.
 *
 */
template <typename Data, uint32_t FifoSize>
class SensorFIFO : public Sensor<Data>
{
protected:
    std::array<Data, FifoSize> lastFifo;
    uint16_t lastFifoLevel          = 1;  //< number of samples in lastFifo
    uint64_t lastInterruptTimestamp = 0;
    uint64_t interruptTimestampDelta =
        0;                        //< delta between previous interrupt
                                  // timestamp and the last received one
    miosix::FastMutex fifoMutex;  // thread safe mutex to read FIFO

public:
    SensorFIFO() {}

    SensorFIFO(SensorFIFO&& other)
        : lastFifo{std::move(other.lastFifo)},
          lastFifoLevel{std::move(other.lastFifoLevel)},
          lastInterruptTimestamp{std::move(other.lastInterruptTimestamp)},
          interruptTimestampDelta{std::move(other.interruptTimestampDelta)},
          fifoMutex{}
    {
    }

    /**
     * @param lastFifoSize output parameter for last FIFO size
     * @return last FIFO sampled from the sensor
     */
    const std::array<Data, FifoSize> getLastFifo(uint16_t& lastFifoSize)
    {
        miosix::Lock<miosix::FastMutex> l(fifoMutex);
        lastFifoSize = lastFifoLevel;
        return lastFifo;
    }

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
