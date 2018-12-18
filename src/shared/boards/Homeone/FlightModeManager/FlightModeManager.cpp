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

#include "boards/Homeone/Events.h"
#include "boards/Homeone/configs/FMMConfig.h"
#include "boards/Homeone/Topics.h"

#include "Debug.h"

namespace HomeoneBoard
{

FlightModeManager::FlightModeManager() : FSM(&FlightModeManager::stateInit)
{
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_ADA);
    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
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

        case EV_INIT_ERROR:
            transition(&FlightModeManager::stateError);
            break;
        case EV_INIT_OK:
            transition(&FlightModeManager::stateDisarmed);
            break;

        default:
            TRACE("FMM stateInit: Event %d not handled.\n", ev.sig);
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

        case EV_TC_BOARD_RESET:
            // TODO: Reboot the board
            break;

        default:
            TRACE("FMM stateTesting: Event %d not handled.\n", ev.sig);
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

        case EV_TC_BOARD_RESET:
            // TODO: Reboot the board
            break;

        default:
            TRACE("FMM stateError: Event %d not handled.\n", ev.sig);
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

        case EV_TC_TEST_MODE:
            transition(&FlightModeManager::stateTesting);
            break;

        case EV_NC_OFFLINE:
        case EV_IGN_OFFLINE:
        case EV_IGN_ABORTED:
            transition(&FlightModeManager::stateError);
            break;

        case EV_TC_ARM:
            transition(&FlightModeManager::stateArmed);
            break;

        default:
            TRACE("FMM stateDisarmed: Event %d not handled.\n", ev.sig);
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

            // Notify everyone that we are armed.
            sEventBroker->post({EV_ARMED}, TOPIC_FLIGHT_EVENTS);

            // Automatically disarm after TIMEOUT_MS_AUTO_DISARM milliseconds.
            delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_ARM}, TOPIC_FMM, TIMEOUT_FMM_AUTO_DISARM);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateArmed\n");

            // Remove disarm timeout
            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_NC_OFFLINE:
        case EV_IGN_OFFLINE:
        case EV_IGN_ABORTED:
            transition(&FlightModeManager::stateError);
            break;

        case EV_TC_DISARM:
        case EV_TIMEOUT_ARM:
            transition(&FlightModeManager::stateDisarmed);
            break;

        case EV_TC_LAUNCH:
            transition(&FlightModeManager::stateLaunching);
            break;

        default:
            TRACE("FMM stateArmed: Event %d not handled.\n", ev.sig);
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

        case EV_UMBILICAL_DETACHED:
            transition(&FlightModeManager::stateAscending);
            break;

        case EV_TC_DISARM:
            transition(&FlightModeManager::stateDisarmed);
            break;

        case EV_IGN_ABORTED:
            transition(&FlightModeManager::stateError);
            break;

        default:
            TRACE("FMM stateLaunching: Event %d not handled.\n", ev.sig);
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

            // Notify liftoff
            sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

            // Set apogee timeout
            delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_APOGEE}, TOPIC_FMM, TIMEOUT_FMM_APOGEE_DETECTION);

            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateAscending\n");

            // Remove apogee timeout
            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_ADA_APOGEE_DETECTED:
        case EV_TIMEOUT_APOGEE:
            transition(&FlightModeManager::stateFirstDescentPhase);
            break;

        default:
            TRACE("FMM stateAscending: Event %d not handled.\n", ev.sig);
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

            // Generate apogee event
            sEventBroker->post({EV_APOGEE}, TOPIC_FLIGHT_EVENTS);

            // Set deployment timeut
            delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_DPL_ALT}, TOPIC_FMM, TIMEOUT_FMM_DPL_ALTITUDE);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateFirstDescentPhase\n");

            // Remove dpl timeout
            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_ADA_DPL_ALT_DETECTED:
        case EV_TIMEOUT_DPL_ALT:
            transition(&FlightModeManager::stateSecondDescentPhase);
            break;

        case EV_TC_MANUAL_MODE:
            transition(&FlightModeManager::stateManualDescent);
            break;

        default:
            TRACE("FMM stateFirstDescentPhase: Event %d not handled.\n", ev.sig);
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

            // Notify deploymnet altitude
            sEventBroker->post({EV_DPL_ALTITUDE}, TOPIC_FLIGHT_EVENTS);

            // Set landing timeout
            delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_END_MISSION}, TOPIC_FMM, TIMEOUT_FMM_END_MISSION);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateSecondDescentPhase\n");

            // Remove end mission timeout
            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_TC_END_MISSION:
        case EV_TIMEOUT_END_MISSION:
            transition(&FlightModeManager::stateLanded);
            break;

        default:
            TRACE("FMM stateSecondDescentPhase: Event %d not handled.\n", ev.sig);
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

            // Set landing timeout
            delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_END_MISSION}, TOPIC_FMM, TIMEOUT_FMM_END_MISSION);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateManualDescent\n");
            break;

        case EV_TC_END_MISSION:
        case EV_TIMEOUT_END_MISSION:
            transition(&FlightModeManager::stateLanded);
            break;

        default:
            TRACE("FMM stateManualDescent: Event %d not handled.\n", ev.sig);
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

            // Notify end of mission
            sEventBroker->post({EV_LANDED}, TOPIC_FLIGHT_EVENTS);
            break;
        case EV_EXIT:
            TRACE("FMM: Exiting stateLanded\n");
            break;

        default:
            TRACE("FMM stateLanded: Event %d not handled.\n", ev.sig);
            break;
    }
}

}  // namespace HomeoneBoard
