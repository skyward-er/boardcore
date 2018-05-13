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
#ifndef SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H
#define SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H

#include "Singleton.h"

#include "events/Event.h"
#include "events/FSM.h"

namespace HomeoneBoard
{
namespace FMM  // Flight Mode Manager
{
/**
 * Implementation of the Flight Mode Manager Finite State Machine
 */
class FlightModeManager : public FSM<FlightModeManager>
{
    friend class Singleton<FlightModeManager>;

public:
    FlightModeManagerFSM();
    ~FlightModeManagerFSM() {}

private:
    // States declarations

    void testing(const Event& e);
    void aborted(const Event& e);
    void disarmed(const Event& e);
    void armed(const Event& e);
    void ascending(const Event& e);
    void apogeeDetection(const Event& e);
    void descendingPhase_1(const Event& e);
    void descendingPhase_2(const Event& e);
    void landed(const Event& e);

    // Event definitions
    // Event ev_ascent_timeout{EV_ASCENT_TIMEOUT};

    // Event ev_apogee_detected{EV_APOGEE_DETECTED};

    // Event ev_main_chute_altitude{EV_MAIN_CHUTE_ALTITUDE};

    // State variables
    uint16_t delayed_event_id = 0;
};
}
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H */
