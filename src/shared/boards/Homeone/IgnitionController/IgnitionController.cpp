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

#include "IgnitionController.h"
#include "boards/Homeone/configs/IgnitionConfig.h"
#include "boards/CanInterfaces.h"
#include "boards/Homeone/Events.h"

namespace HomeoneBoard
{

IgnitionController::IgnitionController(CanBus* canbus)
    : FSM(&IgnitionController::stateIdle), canbus(canbus) 
{
    sEventBroker->subscribe(this, TOPIC_IGNITION);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_CAN);
}

bool IgnitionController::updateIgnBoardStatus(const Event& ev)
{
    const CanbusEvent& cev = static_cast<const CanbusEvent&>(ev);

    if (cev.canTopic == CanInterfaces::CAN_TOPIC_IGNITION)
    {
        memcpy(&status.board_status, cev.payload, sizeof(status.board_status));
        return true;
    }

    return false;
}

void IgnitionController::stateIdle(const Event& ev)
{

    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("IGNCTRL: Entering stateIdle\n");
            status.ctrl_state = IgnitionControllerState::IDLE;
            logger.log(status);

            ev_ign_offline_handle = sEventBroker->postDelayed(
                {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS, TIMEOUT_IGN_OFFLINE);

            // Send first getstatus request
            sEventBroker->post({EV_IGN_GETSTATUS}, TOPIC_IGNITION);

            break;
        case EV_EXIT:
            TRACE("IGNCTRL: Exiting stateIdle\n");
            sEventBroker->removeDelayed(ev_get_status_handle);
            break;

        case EV_IGN_GETSTATUS:
        {
            CanInterfaces::canSendHomeoneCommand(
                            canbus, CanInterfaces::CAN_MSG_REQ_IGN_STATUS);

            ev_get_status_handle = sEventBroker->postDelayed(
                {EV_IGN_GETSTATUS}, TOPIC_IGNITION, INTERVAL_IGN_GET_STATUS);
            break;
        }
        case EV_NEW_CAN_MSG:
        {
            if (updateIgnBoardStatus(ev))
            {
                // Reset the ignition offline timeout
                sEventBroker->removeDelayed(ev_ign_offline_handle);

                ev_ign_offline_handle = sEventBroker->postDelayed(
                    {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS,
                    TIMEOUT_IGN_OFFLINE);

                // Log ignition board status
                logger.log(status);

                if (status.board_status.u1_abort_cmd == 1 ||
                    status.board_status.u2_abort_cmd == 1)
                {
                    // We've had an abort.
                    transition(&IgnitionController::stateAborted);
                    break;
                }
            }
            break;
        }

        case EV_LIFTOFF:
            transition(&IgnitionController::stateEnd);
            break;

        case EV_GS_OFFLINE:
        case EV_TC_ABORT_LAUNCH:
        {
            CanInterfaces::canSendHomeoneCommand(canbus, CanInterfaces::CAN_MSG_ABORT);
            break;
        }
        case EV_LAUNCH:
        {
            const LaunchEvent& lev = static_cast<const LaunchEvent&>(ev);

            CanInterfaces::canSendLaunch(canbus, lev.launchCode);
            break;
        }
        default:
            TRACE("IGNCTRL stateIdle: Event %d not handled.\n", ev.sig);
            break;
    }
}

void IgnitionController::stateAborted(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("IGNCTRL: Entering stateAborted\n");
            status.ctrl_state = IgnitionControllerState::ABORTED;
            logger.log(status);

            sEventBroker->post({EV_IGN_GETSTATUS}, TOPIC_IGNITION);
            break;
        case EV_EXIT:
            TRACE("IGNCTRL: Exiting stateAborted\n");
            break;

        case EV_IGN_GETSTATUS:
        {
            CanInterfaces::canSendHomeoneCommand(
                            canbus, CanInterfaces::CAN_MSG_REQ_IGN_STATUS);
            break;
        }
        // Still handle the abort, just in case we want to send it again
        case EV_TC_ABORT_LAUNCH:
        {
            CanInterfaces::canSendHomeoneCommand(
                            canbus, CanInterfaces::CAN_MSG_ABORT);
            break;
        }

        case EV_NEW_CAN_MSG:
        {
            if (updateIgnBoardStatus(ev))
            {
                // Reset the ignition offline timeout
                sEventBroker->removeDelayed(ev_ign_offline_handle);

                ev_ign_offline_handle = sEventBroker->postDelayed(
                    {EV_IGN_OFFLINE}, TOPIC_FLIGHT_EVENTS,
                    TIMEOUT_IGN_OFFLINE);

                logger.log(status);
            }
            break;
        }

        default:
            TRACE("IGNCTRL stateAborted: Event %d not handled.\n", ev.sig);
            break;
    }
}

void IgnitionController::stateEnd(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("IGNCTRL: Entering stateEnd\n");
            status.ctrl_state = IgnitionControllerState::END;
            logger.log(status);
            break;
        case EV_EXIT:
            TRACE("IGNCTRL: Exiting stateEnd\n");
            break;

        default:
            TRACE("IGNCTRL stateEnd: Event %d not handled.\n", ev.sig);
            break;
    }
}

}  // namespace HomeoneBoard