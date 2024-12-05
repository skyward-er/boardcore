/* Copyright (c) 2021-2024 Skyward Experimental Rocketry
 * Authors: Luca Conterio, Emilio Corigliano
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

#include <Singleton.h>
#include <diagnostic/SkywardStack.h>
#include <utils/KernelTime.h>
#include <utils/TimeUtils.h>

#include "HILPhasesManager.h"
#include "HILTransceiver.h"

namespace Boardcore
{

/**
 * @brief Single interface to the hardware-in-the-loop framework.
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
class HIL : public Boardcore::ActiveObject
{
    using PhasesCallback =
        typename HILPhasesManager<FlightPhases, SimulatorData,
                                  ActuatorData>::PhasesCallback;

public:
    /**
     * @brief Constructor of the HIL framework.
     * @param hilTransceiver The pointer to the already built HILTransceiver.
     * @param hilPhasesManager The pointer to the already built
     * HILPhasesManager.
     * @param updateActuatorData Function which returns the current ActuatorData
     * situation.
     * @param simulationPeriod Period of the simulation [ms].
     */
    HIL(HILTransceiver<FlightPhases, SimulatorData, ActuatorData>*
            hilTransceiver,
        HILPhasesManager<FlightPhases, SimulatorData, ActuatorData>*
            hilPhasesManager,
        std::function<ActuatorData()> updateActuatorData, int simulationPeriod)
        : Boardcore::ActiveObject(Boardcore::STACK_MIN_FOR_SKYWARD,
                                  miosix::PRIORITY_MAX - 1),
          hilTransceiver(hilTransceiver), hilPhasesManager(hilPhasesManager),
          updateActuatorData(updateActuatorData),
          simulationPeriod(simulationPeriod)
    {
    }

    /**
     * @brief Start the needed hardware-in-the-loop components.
     */
    [[nodiscard]] bool start() override
    {
        bool initOk = true;

        if (!hilTransceiver->start())
        {
            LOG_ERR(logger, "hilTransceiver started with errors");
            initOk = false;
        }

        if (!hilPhasesManager->start())
        {
            LOG_ERR(logger, "hilPhasesManager started with errors");
            initOk = false;
        }

        if (!Boardcore::ActiveObject::start())
        {
            LOG_ERR(logger, "hil started with errors");
            initOk = false;
        }

        return initOk;
    }

    void stop()
    {
        hilTransceiver->stop();
        hilPhasesManager->stop();
        Boardcore::ActiveObject::stop();
        LOG_INFO(logger, "HIL framework stopped");
    }

    void waitStartSimulation()
    {
        LOG_INFO(logger, "Waiting for simulation to start...");
        while (!hilPhasesManager->isSimulationRunning())
            miosix::Thread::sleep(1);
    }

    void registerToFlightPhase(const FlightPhases& flag,
                               const PhasesCallback& func)
    {
        hilPhasesManager->registerToFlightPhase(flag, func);
    }

    int getSimulationPeriod() const { return simulationPeriod; }

    int64_t getTimestampSimulatorData() const
    {
        return hilTransceiver->getTimestampSimulatorData();
    }

    const SimulatorData* getSensorData() const
    {
        return hilTransceiver->getSensorData();
    }

protected:
    HILTransceiver<FlightPhases, SimulatorData, ActuatorData>* hilTransceiver;
    HILPhasesManager<FlightPhases, SimulatorData, ActuatorData>*
        hilPhasesManager;

private:
    void run() override
    {
        uint64_t ts = miosix::getTime();
        while (!shouldStop())
        {
            ts += msToNs(simulationPeriod);
            hilTransceiver->setActuatorData(updateActuatorData());
            miosix::Thread::nanoSleepUntil(ts);
        }
    }

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HIL");

    std::function<ActuatorData()> updateActuatorData;
    int simulationPeriod;  // Simulation period in milliseconds
};
}  // namespace Boardcore
