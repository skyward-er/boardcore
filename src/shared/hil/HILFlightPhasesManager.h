/* Copyright (c) 2021-2023 Skyward Experimental Rocketry
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
#include <drivers/timer/TimestampTimer.h>
#include <events/Event.h>
#include <events/EventHandler.h>
#include <hil/HILTransceiver.h>
#include <miosix.h>

#include <iostream>
#include <map>

#include "Events.h"

typedef std::function<void()> TCallback;
class HILTransceiver;

enum class FlightPhases
{
    // simulator flags
    SIM_FLYING,
    SIM_ASCENT,
    SIM_BURNING,
    SIM_AEROBRAKES,
    SIM_PARA1,
    SIM_PARA2,

    // flight flags
    SIMULATION_STARTED,
    CALIBRATION,
    CALIBRATION_OK,
    ARMED,
    LIFTOFF_PIN_DETACHED,
    LIFTOFF,
    AEROBRAKES,
    APOGEE,
    PARA1,
    PARA2,
    SIMULATION_STOPPED
};

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
        printf("@time     : %f [sec]\n", (double)(t - t_start) / 1000000);
        printf("@altitude : %f [m]\n", z);
        printf("@velocity : %f [m/s]\n\n", vz);
    }
};

/**
 * @brief Singleton object that manages all the phases of the simulation.
 * After his instantiation we need to set the source of the current position in
 * order to be able to save the outcomes for each event.
 */
class HILFlightPhasesManager : public Boardcore::EventHandler,
                               public Boardcore::Module
{
    using FlightPhasesFlags = HILConfig::SimulatorData::Flags;

public:
    HILFlightPhasesManager();

    void setCurrentPositionSource(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition);

    bool isFlagActive(FlightPhases flag);

    void registerToFlightPhase(FlightPhases flag, TCallback func);

    void setFlagFlightPhase(FlightPhases flag, bool isEnable);

    virtual void processFlags(FlightPhasesFlags hil_flags);

protected:
    virtual void handleEvent(const Boardcore::Event& e) override;

    virtual void registerOutcomes(FlightPhases phase);

    virtual void printOutcomes();

    /**
     * @brief Updates the flags of the object with the flags sent from matlab
     * and checks for the apogee
     */
    virtual void updateSimulatorFlags(FlightPhasesFlags hil_flags);

    bool isSetTrue(FlightPhases phase);

    bool isSetFalse(FlightPhases phase);

    uint64_t t_start   = 0;
    uint64_t t_liftoff = 0;
    uint64_t t_stop    = 0;
    std::map<FlightPhases, bool> flagsFlightPhases;
    std::map<FlightPhases, bool> prev_flagsFlightPhases;
    std::map<FlightPhases, vector<TCallback>> callbacks;
    std::map<FlightPhases, Outcomes> outcomes;
    std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition;
};
