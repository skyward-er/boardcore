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

#include <utils/ModuleManager/ModuleManager.hpp>

#include "HILPhasesManager.h"
#include "HILTransceiver.h"

/**
 * @brief Single interface to the hardware-in-the-loop framework.
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
class HIL : public Boardcore::Module, public Boardcore::ActiveObject
{
public:
    HIL(HILTransceiver<FlightPhases, SimulatorData, ActuatorData>
            *hilTransceiver,
        HILPhasesManager<FlightPhases, SimulatorData, ActuatorData>
            *hilPhasesManager,
        std::function<ActuatorData()> updateActuatorData, int updatePeriod)
        : Boardcore::ActiveObject(Boardcore::STACK_MIN_FOR_SKYWARD,
                                  miosix::PRIORITY_MAX - 1),
          hilTransceiver(hilTransceiver), hilPhasesManager(hilPhasesManager),
          updateActuatorData(updateActuatorData), updatePeriod(updatePeriod)
    {
        if (!Boardcore::ModuleManager::getInstance().insert<HILTransceiverBase>(
                hilTransceiver))
        {
            LOG_ERR(logger, "HILTransceiver not inserted correctly");
        }

        if (!Boardcore::ModuleManager::getInstance()
                 .insert<HILPhasesManagerBase>(hilPhasesManager))
        {
            LOG_ERR(logger, "HILPhasesManager not inserted correctly");
        }
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
        {
            miosix::Thread::sleep(updatePeriod);
        }
    }

    HILTransceiver<FlightPhases, SimulatorData, ActuatorData> *hilTransceiver;
    HILPhasesManager<FlightPhases, SimulatorData, ActuatorData>
        *hilPhasesManager;

private:
    void run() override
    {
        while (!shouldStop())
        {
            if (hilPhasesManager->isSimulationRunning())
            {
                hilTransceiver->setActuatorData(updateActuatorData());
            }
            miosix::Thread::sleep(updatePeriod);
        }
    }

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("HIL");
    std::function<ActuatorData()> updateActuatorData;
    int updatePeriod;
};