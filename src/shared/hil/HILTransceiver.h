/* Copyright (c) 2020-2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <ActiveObject.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <hil/HILConfig.h>
#include <sensors/HILSensors/HILTimestampManagement.h>
#include <utils/Debug.h>

/**
 * @brief HILTransceiver is a Singleton and provides an easy interface for
 * the control algorithms to send and receive data during a simulation
 */
class HILTransceiver : public Boardcore::ActiveObject
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm
     */
    explicit HILTransceiver(Boardcore::USART &hilSerial);

    /**
     * @brief sets the actuator data and then wakes up the MatlabTransceiver
     * thread in order to send the data back to the simulator (called by the
     * control algorithm)
     * @param actuatorData sets the data that will be sent to the simulator
     */
    void setActuatorData(HILConfig::ActuatorData actuatorData);

    /**
     * @brief returns the reference of the SimulatorData
     *
     * @return reference to the data simulated by matlab
     */
    HILConfig::SimulatorData *getSensorData();

    /**
     * @brief adds to the resetSampleCounter list an object that has to be
     * notified when a new packet of data is arrived from the simulator
     *
     * @param t SimTimestampManagement object
     */
    void addResetSampleCounter(HILTimestampManagement *t);

private:
    /**
     * @brief Waits for the control algorithm(s) to update actuatorData.
     */
    void waitActuatorData();

    void run() override;

    Boardcore::USART &hilSerial;
    bool receivedFirstPacket = false;
    bool updated             = false;
    HILConfig::SimulatorData sensorData;
    HILConfig::ActuatorData actuatorData;
    std::vector<HILTimestampManagement *> sensorsTimestamp;
    miosix::FastMutex mutex;
    miosix::ConditionVariable condVar;
};
