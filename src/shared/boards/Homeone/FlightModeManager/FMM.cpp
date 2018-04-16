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

#include <boards/Homeone/FlightModeManager/FMM.h>
#include "events/EventBroker.h"

#include "boards/Homeone/Events.h"
#include "boards/Homeone/FlightModeManager/FMM_Config.h"
#include "boards/Homeone/Topics.h"

namespace HomeoneBoard
{
namespace FMM
{

FlightModeManagerFSM::FlightModeManagerFSM()
    : FSM(&FlightModeManagerFSM::disarmed)
{
    sEventBroker->subscribe(this, TOPIC_FMM_FSM);
}

void FlightModeManagerFSM::disarmed(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Disarmed state entry\n");
            break;
        case EV_EXIT:
            printf("Disarmed state exit\n");
            break;

        // Transition to armed
        case EV_ARM:
            printf("EEV_ARM\n");
            transition(&FlightModeManagerFSM::armed);
            break;

        // Transition to testing
        case EV_ENABLE_TEST_MODE:
            printf("EV_ENABLE_TEST_MODE\n");
            transition(&FlightModeManagerFSM::testing);
            break;

        // Transition to aborted
        case EV_ABORT_LAUNCH:
            printf("EV_ABORT_LAUNCH\n");
            transition(&FlightModeManagerFSM::aborted);

            // TODO: Start abort procedure
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::armed(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Armed state entry\n");
            break;
        case EV_EXIT:
            printf("Armed state exit\n");
            break;

        // Transition to disarmed
        case EV_DISARM:
            printf("EV_DISARM\n");
            transition(&FlightModeManagerFSM::disarmed);
            break;

        // Transition to ascending
        case EV_UNBILICAL_DETATCHED:
            printf("EV_UNBILICAL_DETATCHED\n");
            transition(&FlightModeManagerFSM::ascending);
            break;

        // Transition to aborted
        case EV_ABORT_LAUNCH:
            printf("EV_ABORT_LAUNCH\n");
            transition(&FlightModeManagerFSM::aborted);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::testing(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Testing state entry\n");
            break;
        case EV_EXIT:
            printf("Testing state exit\n");
            break;

        // Reset the board
        case EV_RESET_BOARD:
            printf("EV_RESET_BOARD\n");
            // TODO: Reset board
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::aborted(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Aborted state entry\n");
            break;
        case EV_EXIT:
            printf("ERROR: Aborted state exit\n");
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::ascending(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Ascending state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                ev_ascent_timeout, TOPIC_FMM_FSM, ASCENDING_TIMEOUT_MS);

            // Start the ada in passive mode
            sEventBroker->post(ev_ada_start, TOPIC_FMM_ADA);
            break;
        case EV_EXIT:
            printf("Ascending state exit\n");

            // Always remove delayed events to avoid errors.
            sEventBroker->removeDelayed(delayed_event_id);

            break;

        // Transition to apogeeDetection
        case EV_FMM_ASCENT_TIMEOUT:
            printf("EV_FMM_ASCENT_TIMEOUT\n");
            transition(&FlightModeManagerFSM::apogeeDetection);
            break;
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::apogeeDetection(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Apogee state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                ev_apogee_detected, TOPIC_FMM_FSM, APOGEE_DETECTION_TIMEOUT_MS);

            // ADA active mode
            sEventBroker->post(ev_ada_active_mode, TOPIC_FMM_ADA);
            break;
        case EV_EXIT:
            printf("Apogee state exit\n");

            // Stop the ada
            sEventBroker->post(ev_ada_stop, TOPIC_FMM_ADA);

            sEventBroker->removeDelayed(delayed_event_id);
            break;

        // Transition to descendingPhase_1
        case EV_APOGEE_DETECTED:
            printf("EV_APOGEE_DETECTED\n");
            transition(&FlightModeManagerFSM::descendingPhase_1);
            break;
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::descendingPhase_1(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Descending_1 state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                ev_main_parachute_deploy, TOPIC_FMM_FSM,
                MAIN_PARACHUTE_DEPLOY_TIMEOUT_MS);

            // TODO: Start altitude detection
            break;
        case EV_EXIT:
            printf("Descending_1 state exit\n");
            // TODO: Stop altitude detection

            sEventBroker->removeDelayed(delayed_event_id);
            break;

        // Transition to descendingPhase_2
        case EV_DEPLOY_MAIN_PARACHUTE:
            printf("EV_FMM_MAIN_PARACHUTE_DEPLOY\n");
            transition(&FlightModeManagerFSM::descendingPhase_2);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::descendingPhase_2(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Descending_2 state entry\n");
            break;
        case EV_EXIT:
            printf("Descending_2 state exit\n");
            break;

        // Transition to landed
        case EV_STOP_SAMPLING:
            printf("Descending_2 state exit\n");

            // Post the event on TOPIC_SAMPLING
            sEventBroker->post(e, TOPIC_SAMPLING);

            transition(&FlightModeManagerFSM::landed);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManagerFSM::landed(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Landed state entry\n");
            break;
        case EV_EXIT:
            printf("ERROR: Landed state exit\n");
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}
}
}
