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

#include <algorithms/AirBrakes/TrajectoryPoint.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/Event.h>
#include <events/EventHandler.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <iostream>
#include <map>

namespace Boardcore
{

struct Outcomes
{
    uint64_t t = 0;
    float z    = 0;
    float vz   = 0;

    Outcomes() : t(0), z(0), vz(0) {}
    Outcomes(float z, float vz)
        : t(TimestampTimer::getTimestamp()), z(z), vz(vz)
    {
    }

    void print(uint64_t t_start) const
    {
        std::cout << "@time     : " << (double)(t - t_start) / 1000000
                  << " [sec]\n";
        std::cout << "@z : " << z << " [m]\n";
        std::cout << "@vz : " << vz << " [m/s]\n\n";
    }
};

class HILPhasesManagerBase : public EventHandler
{
public:
    explicit HILPhasesManagerBase(
        std::function<TimedTrajectoryPoint()> getCurrentPosition)
        : EventHandler(), getCurrentPosition(getCurrentPosition)
    {
    }

    bool isSimulationRunning() const { return (t_start != 0) && (t_stop == 0); }

    virtual void simulationStarted()
    {
        t_start = TimestampTimer::getTimestamp();
        LOG_INFO(logger, "SIMULATION STARTED!");
    }

    virtual void liftoff()
    {
        t_liftoff = TimestampTimer::getTimestamp();
        LOG_INFO(logger, "LIFTOFF!");
    }

    virtual void simulationStopped()
    {
        t_stop = TimestampTimer::getTimestamp();
        LOG_INFO(logger, "SIMULATION STOPPED!");
    }

protected:
    uint64_t t_start   = 0;
    uint64_t t_liftoff = 0;
    uint64_t t_stop    = 0;
    std::function<TimedTrajectoryPoint()> getCurrentPosition;
    PrintLogger logger = Logging::getLogger("HILPhasesManager");
};

/**
 * @brief Singleton object that manages all the phases of the simulation.
 * After his instantiation we need to set the source of the current position in
 * order to be able to save the outcomes for each event.
 */
template <class FlightPhases, class SimulatorData, class ActuatorData>
class HILPhasesManager : public HILPhasesManagerBase
{
public:
    using PhasesCallback = std::function<void()>;

    explicit HILPhasesManager(
        std::function<TimedTrajectoryPoint()> getCurrentPosition)
        : HILPhasesManagerBase(getCurrentPosition)
    {
    }

    bool isFlagActive(const FlightPhases& flag) const
    {
        return flagsFlightPhases.at(flag);
    }

    void registerToFlightPhase(const FlightPhases& flag,
                               const PhasesCallback& func)
    {
        callbacks[flag].push_back(func);
    }

    void setFlagFlightPhase(const FlightPhases& flag, bool isEnable)
    {
        flagsFlightPhases[flag] = isEnable;
    }

    void processFlags(const SimulatorData& hil_flags)
    {
        std::vector<FlightPhases> changed_flags;

        processFlagsImpl(hil_flags, changed_flags);

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<PhasesCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    virtual void printOutcomes() = 0;

protected:
    virtual void processFlagsImpl(const SimulatorData& hil_flags,
                                  std::vector<FlightPhases>& changed_flags) = 0;

    virtual void handleEventImpl(const Event& e,
                                 std::vector<FlightPhases>& changed_flags) = 0;

    void handleEvent(const Boardcore::Event& e) override
    {
        std::vector<FlightPhases> changed_flags;

        handleEventImpl(e, changed_flags);

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<PhasesCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    void registerOutcomes(const FlightPhases& phase)
    {
        TimedTrajectoryPoint temp = getCurrentPosition();
        outcomes[phase]           = Outcomes(temp.z, temp.vz);
    }

    bool isSetTrue(const FlightPhases& phase) const
    {
        return flagsFlightPhases.at(phase) && !prev_flagsFlightPhases.at(phase);
    }

    bool isSetFalse(const FlightPhases& phase) const
    {
        return !flagsFlightPhases.at(phase) && prev_flagsFlightPhases.at(phase);
    }

    std::map<FlightPhases, bool> flagsFlightPhases;
    std::map<FlightPhases, bool> prev_flagsFlightPhases;
    std::map<FlightPhases, std::vector<PhasesCallback>> callbacks;
    std::map<FlightPhases, Outcomes> outcomes;
};
}  // namespace Boardcore