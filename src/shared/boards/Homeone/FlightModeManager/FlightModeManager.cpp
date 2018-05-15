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

#include <boards/Homeone/FlightModeManager/FlightModeManager.h>
#include <events/EventBroker.h>

#include <boards/Homeone/Events.h>
#include <boards/Homeone/FlightModeManager/FMM_Config.h>
#include <boards/Homeone/Topics.h>

namespace HomeoneBoard
{
namespace FMM
{

FlightModeManager::FlightModeManager() : FSM(&FlightModeManager::disarmed)
{
    sEventBroker->subscribe(this, TOPIC_COMMANDS);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
}

void FlightModeManager::disarmed(const Event& e)
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
        case EV_TC_ARM:
            printf("EV_ARM\n");
            transition(&FlightModeManager::armed);
            break;

        // Transition to testing
        case EV_TC_TEST_MODE:
            printf("EV_TC_TEST_MODE\n");
            transition(&FlightModeManager::testing);

            // Send test mode enabled event
            sEventBroker->post(Event{EV_TEST_MODE}, TOPIC_CONFIGURATION);
            break;

        // Transition to aborted
        case EV_ABORT_LAUNCH:
            printf("EV_ABORT_LAUNCH\n");
            transition(&FlightModeManager::aborted);

            // TODO: Do additional abort operations?
            break;

        case EV_TC_START_SAMPLING:
            sEventBroker->post(Event{EV_START_SAMPLING}, TOPIC_CONFIGURATION);
            break;

        case EV_TC_STOP_SAMPLING:
            sEventBroker->post(Event{EV_STOP_SAMPLING}, TOPIC_CONFIGURATION);
            break;

        case EV_TC_ALTIMETER_CALIBRATION:
        {
            // Copy the event and change its id
            Event ev_calib = e;
            ev_calib.sig   = EV_ALTIMETER_CALIBRATION;

            // Post it on the CONFIGURATION topic
            sEventBroker->post(ev_calib, TOPIC_CONFIGURATION);
            break;
        }
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::armed(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            // TODO: Should we start sampling here? in case we forget to send
            // the command

            printf("Armed state entry\n");
            break;
        case EV_EXIT:
            printf("Armed state exit\n");
            break;

        // Transition to disarmed
        case EV_TC_DISARM:
            printf("EV_DISARM\n");
            transition(&FlightModeManager::disarmed);
            break;

        // Transition to ascending
        case EV_UMBILICAL_DISCONNECTED:
            printf("EV_UNBILICAL_DETATCHED\n");
            transition(&FlightModeManager::ascending);
            break;

        // Transition to aborted
        case EV_ABORT_LAUNCH:
            printf("EV_ABORT_LAUNCH\n");
            transition(&FlightModeManager::aborted);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::testing(const Event& e)
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
        case EV_TC_RESET_BOARD:
            printf("EV_RESET_BOARD\n");
            sEventBroker->post(Event{EV_RESET_BOARD}, TOPIC_CONFIGURATION);
            break;

        case EV_TC_NOSECONE_OPEN:
            sEventBroker->post(Event{EV_NOSECONE_OPEN}, TOPIC_CONFIGURATION);
            break;

        case EV_TC_NOSECONE_CLOSE:
            sEventBroker->post(Event{EV_NOSECONE_CLOSE}, TOPIC_CONFIGURATION);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::aborted(const Event& e)
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

void FlightModeManager::ascending(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Ascending state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                Event{EV_ASCENT_TIMEOUT}, TOPIC_FLIGHT_EVENTS,
                ASCENDING_TIMEOUT_MS);

            // ADA passive mode
            sEventBroker->post(Event{EV_ADA_SHADOW_MODE}, TOPIC_FLIGHT_EVENTS);
            break;
        case EV_EXIT:
            printf("Ascending state exit\n");

            // Always remove delayed events to avoid errors.
            sEventBroker->removeDelayed(delayed_event_id);

            break;

        // Transition to apogeeDetection
        case EV_ASCENT_TIMEOUT:
            printf("EV_FMM_ASCENT_TIMEOUT\n");
            transition(&FlightModeManager::apogeeDetection);
            break;
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::apogeeDetection(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Apogee state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                Event{EV_APOGEE_TIMEOUT}, TOPIC_FLIGHT_EVENTS,
                APOGEE_DETECTION_TIMEOUT_MS);

            // ADA active mode
            sEventBroker->post(Event{EV_ADA_ACTIVE_MODE}, TOPIC_FLIGHT_EVENTS);
            break;
        case EV_EXIT:
            printf("Apogee state exit\n");

            // TODO: Stop the ada

            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_APOGEE_TIMEOUT:
            printf("EV_APOGEE_TIMEOUT\n");

            // Post apogee detected event
            sEventBroker->post(Event{EV_APOGEE_DETECTED}, TOPIC_FLIGHT_EVENTS);
            break;

        // Transition to descendingPhase_1
        case EV_APOGEE_DETECTED:
            printf("EV_APOGEE_DETECTED\n");
            transition(&FlightModeManager::descendingPhase_1);
            break;
        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::descendingPhase_1(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("Descending_1 state entry\n");
            // State timeout
            delayed_event_id = sEventBroker->postDelayed(
                Event{EV_DESCENT_TIMEOUT}, TOPIC_FLIGHT_EVENTS,
                MAIN_PARACHUTE_DEPLOY_TIMEOUT_MS);

            // Stop the ADA
            sEventBroker->post(Event{EV_ADA_STOP}, TOPIC_FLIGHT_EVENTS);
            // TODO: Start altitude detection
            break;
        case EV_EXIT:
            printf("Descending_1 state exit\n");
            // TODO: Stop altitude detection

            sEventBroker->removeDelayed(delayed_event_id);
            break;

        case EV_DESCENT_TIMEOUT:
            // Post main chute altitude detected event
            sEventBroker->post(Event{EV_MAIN_CHUTE_ALTITUDE},
                               TOPIC_FLIGHT_EVENTS);
            break;

        // Transition to descendingPhase_2
        case EV_MAIN_CHUTE_ALTITUDE:
            printf("EV_FMM_MAIN_PARACHUTE_DEPLOY\n");
            transition(&FlightModeManager::descendingPhase_2);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::descendingPhase_2(const Event& e)
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
        case EV_TC_STOP_SAMPLING:
            sEventBroker->post(Event{EV_STOP_SAMPLING}, TOPIC_CONFIGURATION);

            transition(&FlightModeManager::landed);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void FlightModeManager::landed(const Event& e)
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
