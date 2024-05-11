/* Copyright (c) 2020-2024 Skyward Experimental Rocketry
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
#include <sensors/HILSensors/HILTimestampManagement.h>
#include <utils/Debug.h>

#include "HIL.h"
#include "drivers/usart/USART.h"

template <class FlightPhases, class SimulatorData, class ActuatorData>
class HIL;

class HILTransceiverBase : public Boardcore::ActiveObject,
                           public Boardcore::Module
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm
     */
    explicit HILTransceiverBase(Boardcore::USART &hilSerial)
        : hilSerial(hilSerial)
    {
    }

    /**
     * @brief adds to the resetSampleCounter list an object that has to be
     * notified when a new packet of data is arrived from the simulator
     *
     * @param t SimTimestampManagement object
     */
    void addResetSampleCounter(HILTimestampManagement *t)
    {
        sensorsTimestamp.push_back(t);
    }

    /**
     * @brief Returns the number of lost updates.
     * @return the number of updates lost due to communication with the
     * simulator.
     */
    int getLostUpdates() { return nLostUpdates; }

protected:
    /**
     * @brief Waits for the control algorithm(s) to update actuatorData.
     */
    void waitActuatorData()
    {
        miosix::Lock<miosix::FastMutex> l(mutex);
        while (!updated)
        {
            condVar.wait(l);
        }
        updated = false;
    }

    Boardcore::USART &hilSerial;
    bool receivedFirstPacket = false;
    bool updated             = false;
    int nLostUpdates         = 0;
    std::vector<HILTimestampManagement *> sensorsTimestamp;
    miosix::FastMutex mutex;
    miosix::ConditionVariable condVar;
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("HILTransceiver");
};

/**
 * @brief HILTransceiver is a Singleton and provides an easy interface for
 * the control algorithms to send and receive data during a simulation
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
class HILTransceiver : public HILTransceiverBase
{
public:
    /**
     * @brief Construct a serial connection attached to a control algorithm
     */
    explicit HILTransceiver(Boardcore::USART &hilSerial)
        : HILTransceiverBase(hilSerial), actuatorData()
    {
    }

    /**
     * @brief sets the actuator data and then wakes up the
     * MatlabTransceiver thread in order to send the data back to the
     * simulator (called by the control algorithm)
     * @param actuatorData sets the data that will be sent to the
     * simulator
     */
    void setActuatorData(ActuatorData actuatorData)
    {
        miosix::Lock<miosix::FastMutex> l(mutex);

        // If already updated increment lost updates
        if (updated)
        {
            nLostUpdates++;
        }

        this->actuatorData = actuatorData;
        updated            = true;
        condVar.signal();
    }

    /**
     * @brief returns the reference of the SimulatorData
     *
     * @return reference to the data simulated by matlab
     */
    const SimulatorData *getSensorData() { return &simulatorData; }

private:
    void run() override;

    SimulatorData simulatorData;
    ActuatorData actuatorData;
};

/**
 * @brief The thread deals with the communication between the simulator and the
 * board.
 *
 * After the first time the data is received, the loop of this thread:
 * - Reads the simulated data and copies them in the SensorData structure;
 * - Notifies every sensor that new data arrived;
 * - Waits for the control algorithms to update the actuator data;
 * - Sends back the value to the simulator.
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
void HILTransceiver<FlightPhases, SimulatorData, ActuatorData>::run()
{
    LOG_INFO(logger, "HIL Transceiver started");
    auto *hilPhasesManager =
        Boardcore::ModuleManager::getInstance()
            .get<HIL<FlightPhases, SimulatorData, ActuatorData>>()
            ->hilPhasesManager;
    bool lostUpdate = false;
    hilSerial.clearQueue();

    while (!shouldStop())
    {
        // Pausing the kernel in order to copy the data in the shared structure
        {
            SimulatorData tempData;
            miosix::led3On();
            if (!hilSerial.readBlocking(&tempData, sizeof(SimulatorData)))
            {
                LOG_ERR(logger, "Failed serial read");
            }
            hilSerial.clearQueue();
            miosix::led3Off();

            miosix::PauseKernelLock kLock;
            simulatorData = tempData;

            if (updated)
            {
                lostUpdate = true;
                updated    = false;  // We want the last computation
            }
        }

        // If this is the first packet to be received, then update the flight
        // phase manager
        if (!receivedFirstPacket)
        {
            receivedFirstPacket = true;
            hilPhasesManager->simulationStarted();
        }

        // Notify all sensors that a new set of data is arrived
        for (auto st : sensorsTimestamp)
            st->resetSampleCounter();

        // Trigger events relative to the flight phases
        hilPhasesManager->processFlags(simulatorData);

        if (lostUpdate)
        {
            // This means also that the number of samples used for the mean sent
            // to the HIL simulator is made up of more than the number of
            // samples we though
            LOG_WARN(logger, "Lost updates");
            lostUpdate = false;
        }

        waitActuatorData();
        miosix::led2On();
        hilSerial.write(&actuatorData, sizeof(ActuatorData));
        miosix::led2Off();
    }
}
