/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include "Singleton.h"

#include "events/FSM.h"
#include "sensors/SensorSampling.h"

// Forward declaration
class TestSensor;

namespace HomeoneBoard
{
namespace Sensors
{

/**
 * Common storage point for all the latest samples of each sensor.
 */
struct SensorData
{
    float testSensorData;
};

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
class SensorManager : public EventHandler, public Singleton<SensorManager>
{
    friend class Singleton<SensorManager>;

public:
    /**
     * Returns a copy of the struct containing the latest available samples from
     * each sensor at the time of the call.
     */
    SensorData getSensorData();

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
         * Handle received events.
         * @param ev Event to handle.
         */
    void handleEvent(const Event& ev);

    /**
     * Adds all the SensorSamplers to the scheduler and begins sampling.
     */
    void startSampling();

    /*
     * Callbacks. These functions are called each time the corresponding
     * SensorSampler has acquired new samples.
     * These functions are called on the scheduler (sampler) thread, so avoid
     * performing non-critical, intensive tasks.
     */

    /**
     * Simple, 10 Hz SensorSampler Callback.
     */
    void onSimple10HZCallback();

    /**
     * Task that sends samples at a fixed rate to the TMTCManager.
     */
    void sendSamplesToTMTC();

    // DMASensorSampler m100HzDMA, m25HzDMA;
    SimpleSensorSampler m10HzSimple;

    // Sensors
    TestSensor* mTestSensor;

    // Sensor data
    SensorData mSensorData{};

    /*
     * Mutex to synchronize mSensorData. We might want to change this:
     * locking and unlocking this mutex each time a new sample is available
     * might be unacceptably slow.
     */
    miosix::FastMutex mSensorDataMutex;
};
}
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGER_H */
