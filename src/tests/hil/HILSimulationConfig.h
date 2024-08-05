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

#include <algorithms/AirBrakes/TrajectoryPoint.h>
#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <events/Event.h>
#include <events/EventHandler.h>
#include <hil/HILPhasesManager.h>
#include <hil/HILTransceiver.h>
#include <math.h>
#include <sensors/HILSensors/IncludeHILSensors.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "algorithms/ADA/ADAData.h"
#include "algorithms/NAS/NAS.h"
#include "algorithms/NAS/NASState.h"
#include "events/EventBroker.h"
#include "sensors/SensorInfo.h"

namespace HILConfig
{

struct SensorConfig : public Boardcore::SensorInfo
{
    SensorConfig(const std::string s, const uint32_t period)
        : Boardcore::SensorInfo{s, period, []() {}, true}
    {
    }
};

/** baudrate of connection */
const int SIM_BAUDRATE = 115200;

/** Period of simulation in milliseconds */
constexpr int SIMULATION_PERIOD = 100;

/** sample frequency of sensor (samples/second) */
constexpr int ACCEL_FREQ        = 100;
constexpr int GYRO_FREQ         = 100;
constexpr int MAGN_FREQ         = 100;
constexpr int IMU_FREQ          = 100;
constexpr int BARO_FREQ         = 50;
constexpr int BARO_CHAMBER_FREQ = 50;
constexpr int PITOT_FREQ        = 20;
constexpr int TEMP_FREQ         = 10;
constexpr int GPS_FREQ          = 10;

/** sensors configuration */
const SensorConfig accelConfig("accel", ACCEL_FREQ);
const SensorConfig gyroConfig("gyro", GYRO_FREQ);
const SensorConfig magnConfig("magn", MAGN_FREQ);
const SensorConfig imuConfig("imu", IMU_FREQ);
const SensorConfig baroConfig("baro", BARO_FREQ);
const SensorConfig pitotConfig("pitot", PITOT_FREQ);
const SensorConfig gpsConfig("gps", GPS_FREQ);
const SensorConfig tempConfig("temp", TEMP_FREQ);

/** Number of samples per sensor at each simulator iteration */
constexpr int N_DATA_ACCEL =
    static_cast<int>((ACCEL_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_GYRO =
    static_cast<int>((GYRO_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_MAGNETO =
    static_cast<int>((MAGN_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_IMU =
    static_cast<int>((IMU_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_BARO =
    static_cast<int>((BARO_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_BARO_CHAMBER =
    static_cast<int>((BARO_CHAMBER_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_PITOT =
    static_cast<int>((PITOT_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_GPS =
    static_cast<int>((GPS_FREQ * SIMULATION_PERIOD) / 1000.0);
constexpr int N_DATA_TEMP =
    static_cast<int>((TEMP_FREQ * SIMULATION_PERIOD) / 1000.0);

struct FlagsHIL
{
    float flag_flight;
    float flag_ascent;
    float flag_burning;
    float flag_airbrakes;
    float flag_para1;
    float flag_para2;

    FlagsHIL(float flag_flight, float flag_ascent, float flag_burning,
             float flag_airbrakes, float flag_para1, float flag_para2)
        : flag_flight(flag_flight), flag_ascent(flag_ascent),
          flag_burning(flag_burning), flag_airbrakes(flag_airbrakes),
          flag_para1(flag_para1), flag_para2(flag_para2)
    {
    }

    FlagsHIL()
        : flag_flight(0.0f), flag_ascent(0.0f), flag_burning(0.0f),
          flag_airbrakes(0.0f), flag_para1(0.0f), flag_para2(0.0f)
    {
    }

    void print()
    {
        printf(
            "flag_flight: %f\n"
            "flag_ascent: %f\n"
            "flag_burning: %f\n"
            "flag_airbrakes: %f\n"
            "flag_para1: %f\n"
            "flag_para2: %f\n",
            flag_flight, flag_ascent, flag_burning, flag_airbrakes, flag_para1,
            flag_para2);
    }
};

/**
 * @brief ADA data sent to the simulator
 */
struct ADAStateHIL
{
    float mslAltitude    = 0;  // Altitude at mean sea level [m].
    float aglAltitude    = 0;  // Altitude above ground level [m].
    float verticalSpeed  = 0;  // Vertical speed [m/s].
    float apogeeDetected = 0;  // Flag if apogee is detected [bool]
    float updating       = 0;  //

    ADAStateHIL()
        : mslAltitude(0), aglAltitude(0), verticalSpeed(0), apogeeDetected(0),
          updating(0)
    {
    }

    void print()
    {
        printf(
            "mslAltitude: %+.3f\n"
            "aglAltitude: %+.3f\n"
            "verticalSpeed: %+.3f\n"
            "apogeeDetected: %+.3f\n"
            "updating: %+.3f\n",
            mslAltitude, aglAltitude, verticalSpeed, apogeeDetected, updating);
    }
};

/**
 * @brief NAS data sent to the simulator
 */
struct NASStateHIL
{
    float n = 0;
    float e = 0;
    float d = 0;

    // Velocity [m/s]
    float vn = 0;
    float ve = 0;
    float vd = 0;

    // Attitude as quaternion
    float qx = 0;
    float qy = 0;
    float qz = 0;
    float qw = 1;

    float updating = 0;  // Flag if apogee is detected [bool]

    NASStateHIL()
        : n(0), e(0), d(0), vn(0), ve(0), vd(0), qx(0), qy(0), qz(0), qw(1),
          updating(0)
    {
    }

    void print()
    {
        printf(
            "n: %+.3f\n"
            "e: %+.3f\n"
            "d: %+.3f\n"
            "vn: %+.3f\n"
            "ve: %+.3f\n"
            "vd: %+.3f\n"
            "qx: %+.3f\n"
            "qy: %+.3f\n"
            "qz: %+.3f\n"
            "qw: %+.3f\n"
            "updating: %+.3f\n",
            n, e, d, vn, ve, vd, qx, qy, qz, qw, updating);
    }
};

/**
 * @brief ABK data sent to the simulator
 */
struct AirBrakesStateHIL
{
    float updating = 0;  // Flag if apogee is detected [bool]

    AirBrakesStateHIL() : updating(0) {}

    void print() { printf("updating: %+.3f\n", updating); }
};

/**
 * @brief MEA data sent to the simulator
 */
struct MEAStateHIL
{
    float correctedPressure = 0;

    float estimatedMass   = 0;
    float estimatedApogee = 0;

    float updating = 0;  // Flag if apogee is detected [bool]

    MEAStateHIL()
        : correctedPressure(0), estimatedMass(0), estimatedApogee(0),
          updating(0)
    {
    }

    void print()
    {
        printf(
            "correctedPressure: %+.3f\n"
            "estimatedMass: %+.3f\n"
            "estimatedApogee: %+.3f\n"
            "updating: %+.3f\n",
            correctedPressure, estimatedMass, estimatedApogee, updating);
    }
};

struct ActuatorsStateHIL
{
    float airbrakesPercentage    = 0;
    float expulsionPercentage    = 0;
    float mainValvePercentage    = 0;
    float ventingValvePercentage = 0;

    ActuatorsStateHIL()
        : airbrakesPercentage(0.0f), expulsionPercentage(0.0f),
          mainValvePercentage(0.0f), ventingValvePercentage(0.0f)
    {
    }

    ActuatorsStateHIL(float airbrakesPercentage, float expulsionPercentage,
                      float mainValvePercentage, float ventingValvePercentage)
        : airbrakesPercentage(airbrakesPercentage),
          expulsionPercentage(expulsionPercentage),
          mainValvePercentage(mainValvePercentage),
          ventingValvePercentage(ventingValvePercentage)
    {
    }

    void print()
    {
        printf(
            "airbrakes: %f perc\n"
            "expulsion: %f perc\n"
            "mainValve: %f perc\n"
            "venting: %f perc\n",
            airbrakesPercentage * 100, expulsionPercentage * 100,
            mainValvePercentage * 100, ventingValvePercentage * 100);
    }
};

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    struct AccelerometerSimulatorData<N_DATA_ACCEL> accelerometer;
    struct GyroscopeSimulatorData<N_DATA_GYRO> gyro;
    struct MagnetometerSimulatorData<N_DATA_MAGNETO> magnetometer;
    struct GPSSimulatorData<N_DATA_GPS> gps;
    struct BarometerSimulatorData<N_DATA_BARO> barometer1, barometer2,
        barometer3;
    struct BarometerSimulatorData<N_DATA_BARO_CHAMBER> pressureChamber;
    struct PitotSimulatorData<N_DATA_PITOT> pitot;
    struct TemperatureSimulatorData<N_DATA_TEMP> temperature;
    struct FlagsHIL flags;
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    ADAStateHIL adaState;
    NASStateHIL nasState;
    AirBrakesStateHIL airBrakesState;
    MEAStateHIL meaState;
    ActuatorsStateHIL actuatorsState;
    FlagsHIL flags;

    ActuatorData()
        : adaState(), nasState(), airBrakesState(), meaState(),
          actuatorsState(), flags()
    {
    }

    void print()
    {
        adaState.print();
        nasState.print();
        airBrakesState.print();
        meaState.print();
        actuatorsState.print();
        flags.print();
    }
};

enum MainFlightPhases
{
    SIM_FLYING,
    SIM_ASCENT,
    SIM_BURNING,
    SIM_AEROBRAKES,
    SIM_PARA1,
    SIM_PARA2,
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

using MainHILAccelerometer = HILAccelerometer<N_DATA_ACCEL>;
using MainHILGyroscope     = HILGyroscope<N_DATA_GYRO>;
using MainHILMagnetometer  = HILMagnetometer<N_DATA_MAGNETO>;
using MainHILGps           = HILGps<N_DATA_GPS>;
using MainHILBarometer     = HILBarometer<N_DATA_BARO>;
using MainHILBarometer     = HILBarometer<N_DATA_BARO_CHAMBER>;
using MainHILPitot         = HILPitot<N_DATA_PITOT>;
using MainHILTemperature   = HILTemperature<N_DATA_TEMP>;

using MainHILTransceiver =
    HILTransceiver<MainFlightPhases, SimulatorData, ActuatorData>;
using MainHIL = HIL<MainFlightPhases, SimulatorData, ActuatorData>;

class MainHILPhasesManager
    : public HILPhasesManager<MainFlightPhases, SimulatorData, ActuatorData>
{
public:
    MainHILPhasesManager(
        std::function<Boardcore::TimedTrajectoryPoint()> getCurrentPosition)
        : HILPhasesManager<MainFlightPhases, SimulatorData, ActuatorData>(
              getCurrentPosition)
    {
        flagsFlightPhases = {{MainFlightPhases::SIM_FLYING, false},
                             {MainFlightPhases::SIM_ASCENT, false},
                             {MainFlightPhases::SIM_BURNING, false},
                             {MainFlightPhases::SIM_AEROBRAKES, false},
                             {MainFlightPhases::SIM_PARA1, false},
                             {MainFlightPhases::SIM_PARA2, false},
                             {MainFlightPhases::SIMULATION_STARTED, false},
                             {MainFlightPhases::CALIBRATION, false},
                             {MainFlightPhases::CALIBRATION_OK, false},
                             {MainFlightPhases::ARMED, false},
                             {MainFlightPhases::LIFTOFF_PIN_DETACHED, false},
                             {MainFlightPhases::LIFTOFF, false},
                             {MainFlightPhases::AEROBRAKES, false},
                             {MainFlightPhases::APOGEE, false},
                             {MainFlightPhases::PARA1, false},
                             {MainFlightPhases::PARA2, false},
                             {MainFlightPhases::SIMULATION_STOPPED, false}};

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    void processFlags(const SimulatorData& simulatorData) override
    {
        updateSimulatorFlags(simulatorData);

        std::vector<MainFlightPhases> changed_flags;

        // set true when the first packet from the simulator arrives
        if (isSetTrue(MainFlightPhases::SIMULATION_STARTED))
        {
            t_start = Boardcore::TimestampTimer::getTimestamp();

            TRACE("[HIL] ------- SIMULATION STARTED ! ------- \n");
            changed_flags.push_back(MainFlightPhases::SIMULATION_STARTED);
        }

        if (flagsFlightPhases[MainFlightPhases::SIM_FLYING])
        {
            if (isSetTrue(MainFlightPhases::SIM_FLYING))
            {
                registerOutcomes(MainFlightPhases::SIM_FLYING);
                TRACE("[HIL] ------- SIMULATOR LIFTOFF ! ------- \n");
                changed_flags.push_back(MainFlightPhases::SIM_FLYING);
            }
        }

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<TCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    void printOutcomes()
    {
        printf("OUTCOMES: (times dt from liftoff)\n\n");
        printf("Simulation time: %.3f [sec]\n\n",
               (double)(t_stop - t_start) / 1000000.0f);

        printf("Motor stopped burning (simulation flag): \n");
        outcomes[MainFlightPhases::SIM_BURNING].print(t_liftoff);

        printf("Airbrakes exit shadowmode: \n");
        outcomes[MainFlightPhases::AEROBRAKES].print(t_liftoff);

        printf("Apogee: \n");
        outcomes[MainFlightPhases::APOGEE].print(t_liftoff);

        printf("Parachute 1: \n");
        outcomes[MainFlightPhases::PARA1].print(t_liftoff);

        printf("Parachute 2: \n");
        outcomes[MainFlightPhases::PARA2].print(t_liftoff);

        printf("Simulation Stopped: \n");
        outcomes[MainFlightPhases::SIMULATION_STOPPED].print(t_liftoff);

        // auto cpuMeter = Boardcore::CpuMeter::getCpuStats();
        // printf("max cpu usage: %+.3f\n", cpuMeter.maxValue);
        // printf("avg cpu usage: %+.3f\n", cpuMeter.mean);
        // printf("min free heap: %+.3lu\n", cpuMeter.minFreeHeap);
        // printf("max free stack:%+.3lu\n", cpuMeter.minFreeStack);
    }

private:
    void handleEvent(const Boardcore::Event& e) override
    {
        std::vector<MainFlightPhases> changed_flags;

        TRACE("%d invalid event\n", e);

        /* calling the callbacks subscribed to the changed flags */
        for (unsigned int i = 0; i < changed_flags.size(); i++)
        {
            std::vector<TCallback> callbacksToCall =
                callbacks[changed_flags[i]];
            for (unsigned int j = 0; j < callbacksToCall.size(); j++)
            {
                callbacksToCall[j]();
            }
        }

        prev_flagsFlightPhases = flagsFlightPhases;
    }

    /**
     * @brief Updates the flags of the object with the flags sent from matlab
     * and checks for the apogee
     */
    void updateSimulatorFlags(const SimulatorData& simulatorData)
    {
        flagsFlightPhases[MainFlightPhases::SIM_ASCENT] =
            simulatorData.flags.flag_ascent;
        flagsFlightPhases[MainFlightPhases::SIM_FLYING] =
            simulatorData.flags.flag_flight;
        flagsFlightPhases[MainFlightPhases::SIM_BURNING] =
            simulatorData.flags.flag_burning;
        flagsFlightPhases[MainFlightPhases::SIM_AEROBRAKES] =
            simulatorData.flags.flag_airbrakes;
        flagsFlightPhases[MainFlightPhases::SIM_PARA1] =
            simulatorData.flags.flag_para1;
        flagsFlightPhases[MainFlightPhases::SIM_PARA2] =
            simulatorData.flags.flag_para2;

        flagsFlightPhases[MainFlightPhases::SIMULATION_STOPPED] =
            isSetFalse(MainFlightPhases::SIM_FLYING) ||
            prev_flagsFlightPhases[MainFlightPhases::SIMULATION_STOPPED];
    }
};

}  // namespace HILConfig
