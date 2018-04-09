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
 * The SensorManager class manages all the sensors connected to the Homeone
 * Board.
 * The class is implemented as a Pseudo-Finite-State-Machine with only one
 * "state". This is done to easily receive and dispatch events thanks to the
 * methods provided by the FSM class.
 * Sensors are grouped by "type" (Simple or DMA) and "sample frequency" and
 * grouped in various SensorSampler objects. These objects are then added to the
 * scheduler that manages the timings for the sampling.
 * After a SensorSampler has finished sampling its sensors, it will call a
 * callback, where these samples can be processed and dispatched.
 */
class SensorManager : public FSM<SensorManager>, public Singleton<SensorManager>
{
    friend class Singleton<SensorManager>;

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
     * Simple, 10 Hz Sensor sampler Callback.
     */
    void onSimple10HZCallback();

    /**
     * Pseud-FSM-State used to handle events
     * @param ev Event to handle.
     */
    void handleEvent(const Event& ev);

    // DMASensorSampler m100HzDMA, m25HzDMA;
    SimpleSensorSampler m10HzSimple;

    // Sensors
    TestSensor* test_sensor;
};
}
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGER_H */
