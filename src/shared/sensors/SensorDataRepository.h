/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rightd
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

#include <Common.h>

#include "SensorData.h"

using namespace miosix;

/**
 * This class aims at decoupling sensors sampling and processing tasks.
 * It acts as a mediator in which the last sample of each sensor is stored.
 */
class SensorDataRepository
{

public:
    /**
     * @brief Constructor. Initialize each element of the samples array.
     *        Create an object for each SensorData subclasses.
     */
    SensorDataRepository();

    /**
     * @brief Destructor.
     */
    ~SensorDataRepository();

    /**
     * @brief Update the sample of type T stored in the repository.
     *
     * @param t   the sample to be used to update the repository.
     */
    template <typename T>
    void addSample(const T& t)
    {
        Lock<FastMutex> lock(samples_array[t.id].mutex);
        *(static_cast<T*>(samples_array[t.id].data)) = t;
    }

    /**
     * @brief Get the last available sample from sensor with given ID.
     *
     * @param sensor_id  ID of the sensor whose sample is requested.
     *
     * @return the last available sample from sensor with given ID.
     */
    template <typename T>
    const T readSample(SensorID sensor_id)
    {
        Lock<FastMutex> lock(samples_array[sensor_id].mutex);
        return *(static_cast<T*>(samples_array[sensor_id].data));
    }

private:
    /**
     * @brief Struct used to avoid synchronized access to the entire samples
     *        array, but synchronize access only to each single element.
     *        The SensorData pointer is used to access the actual data which is
     *        contained in SensorRepo.
     */
    struct SyncSensorData_t
    {
        FastMutex mutex;
        SensorData* data;
    };

    /**
     * @brief Store the last sample of each sensor.
     */
    struct SensorRepo
    {
        SensorData base_data;
        TemperatureData temp_data;
        BarometerData baro_data;
        AccelerometerData accel_data;
        IMU9DofData imu_9dof_data;
        GPSData gps_data;
    };

    SensorRepo
        repo; /**< The actual repository containing the sensors samples */

    SyncSensorData_t
        samples_array[SensorID::LAST_SENSOR_ID]; /**< Last sample for each
                                                    sensor. The sensor ID is
                                                    used as an index to access
                                                    the array */
};