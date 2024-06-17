/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <events/Event.h>

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "Topics.h"

namespace Common
{

enum Events : uint8_t
{
    ABK_DISABLE = Boardcore::EV_FIRST_CUSTOM,
    ABK_OPEN,
    ABK_RESET,
    ABK_SHADOW_MODE_TIMEOUT,
    ABK_WIGGLE,
    ADA_CALIBRATE,
    ADA_PRESS_STAB_TIMEOUT,
    ADA_READY,
    ADA_FORCE_START,
    ADA_FORCE_STOP,
    ADA_SHADOW_MODE_TIMEOUT,
    ADA_APOGEE_DETECTED,
    MEA_SHUTDOWN_DETECTED,
    DPL_CUT_DROGUE,
    DPL_CUT_TIMEOUT,
    DPL_NC_OPEN,
    DPL_NC_RESET,
    DPL_SERVO_ACTUATION_DETECTED,
    DPL_WIGGLE,
    DPL_WES_CAL_DONE,
    CAN_FORCE_INIT,
    CAN_ARM,
    CAN_ENTER_TEST_MODE,
    CAN_EXIT_TEST_MODE,
    CAN_CALIBRATE,
    CAN_DISARM,
    CAN_LIFTOFF,
    CAN_APOGEE_DETECTED,
    CAN_IGNITION,
    FLIGHT_APOGEE_DETECTED,
    FLIGHT_ARMED,
    FLIGHT_DROGUE_DESCENT,
    FLIGHT_DISARMED,
    FLIGHT_DPL_ALT_DETECTED,
    FLIGHT_ERROR_DETECTED,
    FLIGHT_LANDING_DETECTED,
    FLIGHT_LANDING_TIMEOUT,
    FLIGHT_LAUNCH_PIN_DETACHED,
    FLIGHT_LIFTOFF,
    FLIGHT_MOTOR_SHUTDOWN,
    FLIGHT_MISSION_TIMEOUT,
    FLIGHT_NC_DETACHED,
    FLIGHT_WING_DESCENT,
    FMM_ASCENDING,
    FMM_ALGOS_CAL_DONE,
    FMM_STOP_LOGGING,
    FMM_INIT_OK,
    FMM_INIT_ERROR,
    FMM_CALIBRATE,
    FMM_ALGOS_CALIBRATE,
    FMM_SENSORS_CAL_DONE,
    FMM_READY,
    FSR_STATS_TIMEOUT,
    NAS_CALIBRATE,
    NAS_READY,
    NAS_FORCE_START,
    NAS_FORCE_STOP,
    TMTC_ARM,
    TMTC_DISARM,
    TMTC_CALIBRATE,
    TMTC_FORCE_INIT,
    TMTC_FORCE_LAUNCH,
    TMTC_FORCE_LANDING,
    TMTC_FORCE_APOGEE,
    TMTC_FORCE_EXPULSION,
    TMTC_FORCE_DEPLOYMENT,
    TMTC_START_LOGGING,
    TMTC_STOP_LOGGING,
    TMTC_RESET_BOARD,
    TMTC_ENTER_TEST_MODE,
    TMTC_EXIT_TEST_MODE,
    TMTC_START_RECORDING,
    TMTC_STOP_RECORDING,
    MOTOR_START_TARS,
    MOTOR_STOP_TARS,
    MOTOR_OPEN_VENTING_VALVE,
    MOTOR_CLOSE_VENTING_VALVE,
    MOTOR_OPEN_FILLING_VALVE,
    MOTOR_CLOSE_FILLING_VALVE,
    MOTOR_OPEN_RELEASE_VALVE,
    MOTOR_CLOSE_RELEASE_VALVE,
    MOTOR_DISCONNECT,
    MOTOR_IGNITION,
    MOTOR_OPEN_FEED_VALVE,
    MOTOR_CLOSE_FEED_VALVE,
    MOTOR_MANUAL_ACTION,
    MOTOR_OPEN_OXIDANT,
    MOTOR_SHADOW_MODE_TIMEOUT,
    TARS_WASHING_DONE,
    TARS_CHECK_PRESSURE_STABILIZE,
    TARS_PRESSURE_STABILIZED,
    TARS_FILLING_DONE,
    TARS_REFINING_DONE,
    ALTITUDE_TRIGGER_ALTITUDE_REACHED,
    LAST_EVENT
};

inline std::string getEventString(uint8_t event)
{
    static const std::map<uint8_t, std::string> event_string_map{
        {ABK_DISABLE, "ABK_DISABLE"},
        {ABK_OPEN, "ABK_OPEN"},
        {ABK_RESET, "ABK_RESET"},
        {ABK_SHADOW_MODE_TIMEOUT, "ABK_SHADOW_MODE_TIMEOUT"},
        {ABK_WIGGLE, "ABK_WIGGLE"},
        {ADA_CALIBRATE, "ADA_CALIBRATE"},
        {ADA_PRESS_STAB_TIMEOUT, "ADA_PRESS_STAB_TIMEOUT"},
        {ADA_READY, "ADA_READY"},
        {ADA_FORCE_START, "ADA_FORCE_START"},
        {ADA_FORCE_STOP, "ADA_FORCE_STOP"},
        {ADA_SHADOW_MODE_TIMEOUT, "ADA_SHADOW_MODE_TIMEOUT"},
        {ADA_APOGEE_DETECTED, "ADA_APOGEE_DETECTED"},
        {MEA_SHUTDOWN_DETECTED, "MEA_SHUTDOWN_DETECTED"},
        {DPL_CUT_DROGUE, "DPL_CUT_DROGUE"},
        {DPL_CUT_TIMEOUT, "DPL_CUT_TIMEOUT"},
        {DPL_NC_OPEN, "DPL_NC_OPEN"},
        {DPL_NC_RESET, "DPL_NC_RESET"},
        {DPL_SERVO_ACTUATION_DETECTED, "DPL_SERVO_ACTUATION_DETECTED"},
        {DPL_WIGGLE, "DPL_WIGGLE"},
        {DPL_WES_CAL_DONE, "DPL_WES_CAL_DONE"},
        {CAN_FORCE_INIT, "CAN_FORCE_INIT"},
        {CAN_ARM, "CAN_ARM"},
        {CAN_ENTER_TEST_MODE, "CAN_ENTER_TEST_MODE"},
        {CAN_EXIT_TEST_MODE, "CAN_EXIT_TEST_MODE"},
        {CAN_CALIBRATE, "CAN_CALIBRATE"},
        {CAN_DISARM, "CAN_DISARM"},
        {CAN_LIFTOFF, "CAN_LIFTOFF"},
        {CAN_APOGEE_DETECTED, "CAN_APOGEE_DETECTED"},
        {CAN_IGNITION, "CAN_IGNITION"},
        {FLIGHT_APOGEE_DETECTED, "FLIGHT_APOGEE_DETECTED"},
        {FLIGHT_ARMED, "FLIGHT_ARMED"},
        {FLIGHT_DROGUE_DESCENT, "FLIGHT_DROGUE_DESCENT"},
        {FLIGHT_DISARMED, "FLIGHT_DISARMED"},
        {FLIGHT_DPL_ALT_DETECTED, "FLIGHT_DPL_ALT_DETECTED"},
        {FLIGHT_ERROR_DETECTED, "FLIGHT_ERROR_DETECTED"},
        {FLIGHT_LANDING_DETECTED, "FLIGHT_LANDING_DETECTED"},
        {FLIGHT_LAUNCH_PIN_DETACHED, "FLIGHT_LAUNCH_PIN_DETACHED"},
        {FLIGHT_LIFTOFF, "FLIGHT_LIFTOFF"},
        {FLIGHT_MOTOR_SHUTDOWN, "FLIGHT_MOTOR_SHUTDOWN"},
        {FLIGHT_LANDING_TIMEOUT, "FLIGHT_LANDING_TIMEOUT"},
        {FLIGHT_NC_DETACHED, "FLIGHT_NC_DETACHED"},
        {FLIGHT_MISSION_TIMEOUT, "FLIGHT_MISSION_TIMEOUT"},
        {FLIGHT_WING_DESCENT, "FLIGHT_WING_DESCENT"},
        {FMM_ASCENDING, "FMM_ASCENDING"},
        {FMM_ALGOS_CAL_DONE, "FMM_ALGOS_CAL_DONE"},
        {FMM_STOP_LOGGING, "FMM_STOP_LOGGING"},
        {FMM_INIT_OK, "FMM_INIT_OK"},
        {FMM_INIT_ERROR, "FMM_INIT_ERROR"},
        {FMM_CALIBRATE, "FMM_CALIBRATE"},
        {FMM_ALGOS_CALIBRATE, "FMM_ALGOS_CALIBRATE"},
        {FMM_SENSORS_CAL_DONE, "FMM_SENSORS_CAL_DONE"},
        {FMM_READY, "FMM_READY"},
        {FSR_STATS_TIMEOUT, "FSR_STATS_TIMEOUT"},
        {NAS_CALIBRATE, "NAS_CALIBRATE"},
        {NAS_FORCE_START, "NAS_FORCE_START"},
        {NAS_FORCE_STOP, "NAS_FORCE_STOP"},
        {NAS_READY, "NAS_READY"},
        {TMTC_ARM, "TMTC_ARM"},
        {TMTC_DISARM, "TMTC_DISARM"},
        {TMTC_CALIBRATE, "TMTC_CALIBRATE"},
        {TMTC_FORCE_INIT, "TMTC_FORCE_INIT"},
        {TMTC_FORCE_LAUNCH, "TMTC_FORCE_LAUNCH"},
        {TMTC_FORCE_LANDING, "TMTC_FORCE_LANDING"},
        {TMTC_FORCE_APOGEE, "TMTC_FORCE_APOGEE"},
        {TMTC_FORCE_EXPULSION, "TMTC_FORCE_EXPULSION"},
        {TMTC_FORCE_DEPLOYMENT, "TMTC_FORCE_DEPLOYMENT"},
        {TMTC_START_LOGGING, "TMTC_START_LOGGING"},
        {TMTC_STOP_LOGGING, "TMTC_STOP_LOGGING"},
        {TMTC_RESET_BOARD, "TMTC_RESET_BOARD"},
        {TMTC_ENTER_TEST_MODE, "TMTC_ENTER_TEST_MODE"},
        {TMTC_EXIT_TEST_MODE, "TMTC_EXIT_TEST_MODE"},
        {TMTC_START_RECORDING, "TMTC_START_RECORDING"},
        {TMTC_STOP_RECORDING, "TMTC_STOP_RECORDING"},
        {MOTOR_START_TARS, "MOTOR_START_TARS"},
        {MOTOR_STOP_TARS, "MOTOR_STOP_TARS"},
        {MOTOR_OPEN_VENTING_VALVE, "MOTOR_OPEN_VENTING_VALVE"},
        {MOTOR_CLOSE_VENTING_VALVE, "MOTOR_CLOSE_VENTING_VALVE"},
        {MOTOR_OPEN_FILLING_VALVE, "MOTOR_OPEN_FILLING_VALVE"},
        {MOTOR_CLOSE_FILLING_VALVE, "MOTOR_CLOSE_FILLING_VALVE"},
        {MOTOR_OPEN_RELEASE_VALVE, "MOTOR_OPEN_RELEASE_VALVE"},
        {MOTOR_CLOSE_RELEASE_VALVE, "MOTOR_CLOSE_RELEASE_VALVE"},
        {MOTOR_DISCONNECT, "MOTOR_DISCONNECT"},
        {MOTOR_IGNITION, "MOTOR_IGNITION"},
        {MOTOR_OPEN_FEED_VALVE, "MOTOR_OPEN_FEED_VALVE"},
        {MOTOR_CLOSE_FEED_VALVE, "MOTOR_CLOSE_FEED_VALVE"},
        {MOTOR_MANUAL_ACTION, "MOTOR_MANUAL_ACTION"},
        {MOTOR_OPEN_OXIDANT, "MOTOR_OPEN_OXIDANT"},
        {MOTOR_SHADOW_MODE_TIMEOUT, "MOTOR_SHADOW_MODE_TIMEOUT"},
        {TARS_WASHING_DONE, "TARS_WASHING_DONE"},
        {TARS_CHECK_PRESSURE_STABILIZE, "TARS_CHECK_PRESSURE_STABILIZE"},
        {TARS_PRESSURE_STABILIZED, "TARS_PRESSURE_STABILIZED"},
        {TARS_FILLING_DONE, "TARS_FILLING_DONE"},
        {TARS_REFINING_DONE, "TARS_REFINING_DONE"},
        {ALTITUDE_TRIGGER_ALTITUDE_REACHED,
         "ALTITUDE_TRIGGER_ALTITUDE_REACHED"},
        {LAST_EVENT, "LAST_EVENT"},
    };

    auto it = event_string_map.find(event);
    return it == event_string_map.end() ? "EV_UNKNOWN" : it->second;
}

struct LiftoffEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

struct ApogeeEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

struct NoseconeEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

struct ExpulsionEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

struct MainEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

struct LandingEvent
{
    uint64_t timestamp;

    static std::string header() { return "timestamp\n"; }

    void print(std::ostream& os) const { os << timestamp << "\n"; }
};

}  // namespace Common
