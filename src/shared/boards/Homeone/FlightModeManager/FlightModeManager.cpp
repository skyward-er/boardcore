/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <boards/Homeone/FlightModeManager/FlightModeManager.h>
#include <events/EventBroker.h>

#include <boards/Homeone/Events.h>
#include <boards/Homeone/FlightModeManager/FMM_Config.h>
#include <boards/Homeone/Topics.h>

#include "debug.h"

namespace HomeoneBoard
{
namespace FMM
{

FlightModeManager::FlightModeManager() : FSM(&FlightModeManager::state_disarmed)
{
    // sEventBroker->subscribe(this, TOPIC_COMMANDS);
    // sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
}

void FlightModeManager::stateInit(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateInit\n");
            status.state = FMMState::INIT;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateInit\n");
            break;

        default:
            TRACE("stateInit: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateTesting(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateTesting\n");
            status.state = FMMState::TESTING;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateTesting\n");
            break;

        default:
            TRACE("stateTesting: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateError(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateError\n");
            status.state = FMMState::ERROR;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateError\n");
            break;

        default:
            TRACE("stateError: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateAborted(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateAborted\n");
            status.state = FMMState::ABORTED;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateAborted\n");
            break;

        default:
            TRACE("stateAborted: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateDisarmed(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateDisarmed\n");
            status.state = FMMState::DISARMED;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateDisarmed\n");
            break;

        default:
            TRACE("stateDisarmed: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateArmed(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateArmed\n");
            status.state = FMMState::ARMED;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateArmed\n");
            break;

        default:
            TRACE("stateArmed: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateLaunching(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateLaunching\n");
            status.state = FMMState::LAUNCHING;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateLaunching\n");
            break;

        default:
            TRACE("stateLaunching: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateAscending(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateAscending\n");
            status.state = FMMState::ASCENDING;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateAscending\n");
            break;

        default:
            TRACE("stateAscending: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateFirstDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateFirstDescentPhase\n");
            status.state = FMMState::FIRST_DESCENT_PHASE;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateFirstDescentPhase\n");
            break;

        default:
            TRACE("stateFirstDescentPhase: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateSecondDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateSecondDescentPhase\n");
            status.state = FMMState::SECOND_DESCENT_PHASE;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateSecondDescentPhase\n");
            break;

        default:
            TRACE("stateSecondDescentPhase: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateManualDescent(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateManualDescent\n");
            status.state = FMMState::MANUAL_DESCENT;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateManualDescent\n");
            break;

        default:
            TRACE("stateManualDescent: Event %d not handled.\n", ev.sig);
            break;
    }
}

void FlightModeManager::stateLanded(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("FMM: Entering stateLanded\n");
            status.state = FMMState::LANDED;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateLanded\n");
            break;

        default:
            TRACE("stateLanded: Event %d not handled.\n", ev.sig);
            break;
    }
}

}  // namespace FMM
}  // namespace HomeoneBoard
