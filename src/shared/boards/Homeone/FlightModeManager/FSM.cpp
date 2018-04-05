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

#include "events/EventBroker.h"

#include "boards/Homeone/Events.h"
#include "boards/Homeone/FlightModeManager/FMM_Config.h"
#include "boards/Homeone/FlightModeManager/FSM.h"
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
        case EV_FMM_ARM:  // Transition to armed
            printf("EV_FMM_ARM\n");
            transition(&FlightModeManagerFSM::armed);
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
        case EV_FMM_DISARM:
            printf("EV_FMM_DISARM\n");
            transition(&FlightModeManagerFSM::disarmed);
            break;
        case EV_FMM_LAUNCH:
            printf("EV_FMM_LAUNCH\n");
            transition(&FlightModeManagerFSM::ascending);
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

            // TODO: Start ADA in passive mode
            break;
        case EV_EXIT:
            printf("Ascending state exit\n");
            sEventBroker->removeDelayed(delayed_event_id);
            break;
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
                ev_apogee, TOPIC_FMM_FSM, APOGEE_DETECTION_TIMEOUT_MS);

            // TODO: ADA active mode
            break;
        case EV_EXIT:
            printf("Apogee state exit\n");
            // TODO: Stop ADA

            sEventBroker->removeDelayed(delayed_event_id);
            break;
        case EV_FMM_APOGEE:
            printf("EV_FMM_APOGEE\n");
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
            // TODO: Activate thermocutters
            // TODO: Stop altitude detection

            sEventBroker->removeDelayed(delayed_event_id);
            break;
        case EV_FMM_MAIN_PARACHUTE_DEPLOY:
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
        default:
            printf("Unknown event received.\n");
            break;
    }
}
}
}
