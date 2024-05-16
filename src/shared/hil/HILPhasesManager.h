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
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Boardcore
{

struct Outcomes
{
    uint64_t t = 0;
    float z    = 0;
    float vz   = 0;

    Outcomes() : t(0), z(0), vz(0) {}
    Outcomes(float z, float vz)
        : t(Boardcore::TimestampTimer::getTimestamp()), z(z), vz(vz)
    {
    }

    void print(uint64_t t_start) const
    {
        TRACE("@time     : %f [sec]\n", (double)(t - t_start) / 1000000);
        TRACE("@altitude : %f [m]\n", z);
        TRACE("@velocity : %f [m/s]\n\n", vz);
    }
};

class HILPhasesManagerBase : public Boardcore::EventHandler,
                             public Boardcore::Module
{
public:
    explicit HILPhasesManagerBase(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : Boardcore::EventHandler(), getCurrentPosition(getCurrentPosition)
    {
    }

    bool isSimulationRunning() { return (t_start != 0) && (t_stop == 0); }

    virtual void simulationStarted()
    {
        t_start = Boardcore::TimestampTimer::getTimestamp();
        LOG_INFO(logger, "SIMULATION STARTED!");
    }

    virtual void liftoff()
    {
        t_liftoff = Boardcore::TimestampTimer::getTimestamp();
        LOG_INFO(logger, "LIFTOFF!");
    }

    virtual void simulationStopped()
    {
        t_stop = Boardcore::TimestampTimer::getTimestamp();
        LOG_INFO(logger, "SIMULATION STOPPED!");
    }

protected:
    uint64_t t_start   = 0;
    uint64_t t_liftoff = 0;
    uint64_t t_stop    = 0;
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition;
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("HILPhasesManager");
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
    typedef std::function<void()> TCallback;

    explicit HILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : HILPhasesManagerBase(getCurrentPosition)
    {
    }

    bool isFlagActive(FlightPhases flag) { return flagsFlightPhases[flag]; }

    void registerToFlightPhase(FlightPhases flag, TCallback func)
    {
        callbacks[flag].push_back(func);
    }

    void setFlagFlightPhase(FlightPhases flag, bool isEnable)
    {
        flagsFlightPhases[flag] = isEnable;
    }

    virtual void processFlags(const SimulatorData& hil_flags) = 0;

    virtual void printOutcomes() = 0;

protected:
    virtual void handleEvent(const Boardcore::Event& e) = 0;

    void registerOutcomes(FlightPhases phase)
    {
        Boardcore::TimedTrajectoryPoint temp = getCurrentPosition();
        outcomes[phase]                      = Outcomes(temp.z, temp.vz);
    }

    bool isSetTrue(FlightPhases phase)
    {
        return flagsFlightPhases[phase] && !prev_flagsFlightPhases[phase];
    }

    bool isSetFalse(FlightPhases phase)
    {
        return !flagsFlightPhases[phase] && prev_flagsFlightPhases[phase];
    }

    std::map<FlightPhases, bool> flagsFlightPhases;
    std::map<FlightPhases, bool> prev_flagsFlightPhases;
    std::map<FlightPhases, std::vector<TCallback>> callbacks;
    std::map<FlightPhases, Outcomes> outcomes;
};
}  // namespace Boardcore