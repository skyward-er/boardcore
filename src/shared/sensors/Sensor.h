/* Sensors Base Classes
 *
 * Copyright (c) 2016-2020 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <array>
#include <type_traits>

#include "SensorData.h"

/**
 * @brief Check that a given type has a method called `getData()` and that the
 * return type of this method is a subclass of the expected data type.
 */
template <class T, class ExpectedDataType>
struct checkIfProduces
    : std::is_base_of<ExpectedDataType,
                      typename std::remove_reference<decltype(
                          std::declval<T>().getLastSample())>::type>
{
};

/**
 * @brief Base abstract class for sensor drivers.
 */
class AbstractSensor
{
protected:
    SensorErrors last_error = SensorErrors::NO_ERRORS;

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
    SensorErrors getLastError() { return last_error; };
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
protected:
    Data last_sample;

    /**
     * @brief Read a data sample from the sensor.
     *        In case of errors, the method should return the last
     *        available correct sample.
     * @return sensor data sample
     */
    virtual Data sampleImpl() = 0;

public:
    virtual ~Sensor() {}

    void sample() override { last_sample = sampleImpl(); }

    /**
     * @return last available sample from this sensor
     */
    virtual const Data& getLastSample() { return last_sample; }
};

/**
 * @brief Interface for sensor that implement a FIFO.
 */
template <typename Data, uint32_t FifoSize>
class SensorFIFO : public Sensor<Data>
{
protected:
    std::array<Data, FifoSize> last_fifo;
    uint8_t last_fifo_level = 1; /**< number of samples in last_fifo */

    uint64_t last_interrupt_us = 0; /**< last interrupt timestamp */
    uint64_t dt_interrupt = 0; /**< delta between previous interrupt timestamp
                                  and the last received one */

public:
    /**
     * @return last FIFO sampled from the sensor
     */
    const std::array<Data, FifoSize>& getLastFifo() { return last_fifo; }

    /**
     * @param i   index of the requested item inside the FIFO
     *
     * @return the i-th element of the FIFO
     */
    const Data& getFifoElement(uint32_t i) const { return last_fifo[i]; }

    /**
     * @return number of elements in the last FIFO sampled from the sensor
     */
    uint8_t getLastFifoSize() const { return last_fifo_level; }

    /**
     * @brief Called by the interrupt handling routine: provides the timestamp
     *        of the last interrupt (if FIFO is disabled) or the last watermark
     *        interrupt (if FIFO enabled)
     *
     * @param ts Timestamp of the lasts interrupt, in microseconds
     */
    inline void IRQupdateTimestamp(uint64_t ts)
    {
        dt_interrupt      = ts - last_interrupt_us;
        last_interrupt_us = ts;
    }
};
