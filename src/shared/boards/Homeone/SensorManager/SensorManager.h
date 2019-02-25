/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGER_H
#define SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGER_H

#include <vector>

#include "Singleton.h"
#include "events/Scheduler.h"

#include "boards/Homeone/configs/SensorManagerConfig.h"
#include "events/FSM.h"
#include <boards/Homeone/LogProxy/LogProxy.h>
#include "sensors/SensorSampling.h"

#include "SensorManagerData.h"

using miosix::PauseKernelLock;
using std::vector;

// Forward declarations
class TestSensor;

template <typename BusI2C, typename BusyPin, typename CONVST>
class AD7994;

template <typename BusSPI>
class MPU9250;

template <typename BusSPI>
class MAX21105;

template <typename BusSPI>
class ADIS16405;


namespace HomeoneBoard
{
// Type definitions
typedef AD7994<busI2C1, ad7994_busy_pin, ad7994_nconvst> AD7994Type;
typedef MPU9250<spiMPU9250> MPU9250Type;
typedef MAX21105<spiMAX21105> MAX21105Type;
// typedef ADIS16405<spiADIS16405> ADIS16405Type;

/**
 * The SensorManager class manages all the sensors connected to the Homeone
 * Board.
 *
 * Sensors are grouped by "type" (Simple or DMA) and "sample frequency" and
 * grouped in various SensorSampler objects. These objects are then added to the
 * scheduler that manages the timings for the sampling.
 * After a SensorSampler has finished sampling its sensors, it will call a
 * callback, where these samples can be processed and dispatched.
 */
class SensorManager : public FSM<SensorManager>, public Singleton<SensorManager>
{
    friend class Singleton<SensorManager>;

public:
    vector<TaskStatResult> getSchedulerStats()
    {
        PauseKernelLock l;  // Prevent any context-switch.
        return scheduler_stats;
    }

    SensorManagerStatus getStatus()
    {
        PauseKernelLock l;  // Prevent any context-switch.
        return status;
    }

private:
    SensorManager();
    ~SensorManager(){};

    /**
     * Initialize all the sensors.
     */
    void initSensors();

    /**
     * Initialize the samplers.
     */
    void initSamplers();

    /**
     * @brief Sensor manager state machine entry state
     *
     */
    void stateIdle(const Event& ev);

    /**
     * @brief Sensor manager state machine sampling state
     *
     */
    void stateLogging(const Event& ev);

    /**
     * Adds all the SensorSamplers to the scheduler and begins sampling.
     */
    void startSampling();

    /*
     * Callbacks. These functions are called each time the corresponding
     * SensorSampler has acquired new samples.
     * These functions are called on the scheduler (sampler) thread, so avoid
     * performing non-critical and intensive tasks.
     */

    /**
     * @brief Simple, 20 Hz SensorSampler Callback.
     */
    void onSimple20HZCallback();

    /**
     * @brief DMA, 500 Hz SensorSampler Callback.
     */
    void onDMA500HZCallback();

    // Sensor samplers
    SimpleSensorSampler sampler_20hz_simple;
    DMASensorSampler sampler_500hz_dma;

    // Sensors
    TestSensor* sensor_test;  // TODO: Remove test sensor
    AD7994Type* adc_ad7994;
    MAX21105Type* imu_max21105;
    MPU9250Type* imu_mpu9250;
    // ADIS16405Type* imu_adis16405;

    // Stats & status
    vector<TaskStatResult> scheduler_stats;
    SensorManagerStatus status;

    // Logger ref
    LoggerProxy& logger;

    bool enable_sensor_logging = false;
};

}  // namespace HomeoneBoard

#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGER_H */