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

#include "FMMStatus.h"
#include "events/Event.h"
#include "events/FSM.h"
#include <boards/Homeone/LogProxy/LogProxy.h>

#include <miosix.h>

using miosix::FastMutex;
using miosix::Lock;

namespace HomeoneBoard
{

/**
 * Implementation of the Flight Mode Manager Finite State Machine
 */
class FlightModeManager : public FSM<FlightModeManager>,
                          public Singleton<FlightModeManager>
{
    friend class Singleton<FlightModeManager>;

public:
    FMMStatus getStatus()
    {
        Lock<FastMutex> lock(mtx_status);
        return status;
    }

private:
    FlightModeManager();
    ~FlightModeManager() {}

    // State declarations
    void stateInit(const Event& ev);
    void stateTesting(const Event& ev);
    void stateError(const Event& ev);
    void stateDisarmed(const Event& ev);
    void stateArmed(const Event& ev);
    void stateLaunching(const Event& ev);
    void stateAscending(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateSecondDescentPhase(const Event& ev);
    void stateManualDescent(const Event& ev);
    void stateLanded(const Event& ev);

    // Other methods

    // State variables
    uint16_t delayed_event_id = 0;

    FastMutex mtx_status;
    FMMStatus status;

    LoggerProxy& logger = *(LoggerProxy::getInstance());
};

}  // namespace HomeoneBoard

#endif /* SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H */
