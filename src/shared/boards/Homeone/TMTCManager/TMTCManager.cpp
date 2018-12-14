/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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

#include "TMTCManager.h"
#include "boards/Homeone/Topics.h"
#include "TCHandler.h"

namespace HomeoneBoard
{

TMTCManager::TMTCManager() : FSM(&TMTCManager::stateIdle)
{
    device   = new Gamma868("/dev/radio");

    mavManager = new MavManager();
    mavManager->addSender(device, 250);
    mavManager->addReceiver(device, mavManager->getSender(0), 
                                    &TCHandler::handleMavlinkMessage);

    TRACE("[TMTC] Created TMTCManager\n");

    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TMTC);
}


TMTCManager::~TMTCManager()
{
    delete device;
    delete sender;
    delete receiver;
}


void TMTCManager::stateIdle(const Event& ev)
{
    switch(ev.sig) 
    {
        case EV_ENTRY:
            TRACE("[TMTC] Entering stateIdle\n");
            g_gsOfflineEvId = sEventBroker->postDelayed(Event{EV_GS_OFFLINE}, 
                                                            GS_OFFLINE_TIMEOUT);
            break;

        case EV_LIFTOFF:
            TRACE("[TMTC] Liftoff signal received\n");
            transition(&TMTCManager::stateHighRateTM);
            break;

        case EV_EXIT:
            TRACE("[TMTC] Exiting stateIdle\n");
            break;

        default:
            TRACE("[TMTC] Event not handled\n");
            break;
    }
}


void TMTCManager::stateHighRateTM(const Event& ev)
{
    switch(ev.sig) 
    {
        case EV_ENTRY:
            TRACE("[TMTC] Entering stateHighRateTM\n");
            sEventBroker->postDelayed(Event{EV_SEND_HR_TM}, TOPIC_TMTC, HR_TM_TIMEOUT);
            break;

        case EV_SEND_HR_TM: {
            TRACE("[TMTC] Sending HR telemetry\n");

            mavlink_message_t telem = TMBuilder::buildTelemetry(MAV_HR_TM_ID);
            sender->enqueueMsg(telem);

            sEventBroker->postDelayed(Event{EV_SEND_HR_TM}, TOPIC_TMTC, HR_TM_TIMEOUT);
            break;
        }

        case EV_APOGEE:
            TRACE("[TMTC] Apogee signal received\n");
            transition(&TMTCManager::stateLowRateTM);
            break;

        case EV_EXIT:
            TRACE("[TMTC] Exiting stateHighRateTM\n");
            break;

        default:
            TRACE("[TMTC] Event not handled\n");
            break;
    }
}


void TMTCManager::stateLowRateTM(const Event& ev)
{
    switch(ev.sig) 
    {
        case EV_ENTRY:
            TRACE("[TMTC] Entering stateLowRateTM\n");
            sEventBroker->postDelayed(Event{EV_SEND_LR_TM}, TOPIC_TMTC, LR_TM_TIMEOUT);
            break;

        case EV_SEND_LR_TM: {
            TRACE("[TMTC] Sending LR telemetry\n");

            mavlink_message_t telem = TMBuilder::buildTelemetry(MAV_LR_TM_ID);
            sender->enqueueMsg(telem);

            sEventBroker->postDelayed(Event{EV_SEND_LR_TM}, TOPIC_TMTC, LR_TM_TIMEOUT);
            break;
        }

        case EV_EXIT:
            TRACE("[TMTC] Exiting stateLowRateTM\n");
            break;

        default:
            TRACE("[TMTC] Event not handled\n");
            break;
    }
}

} /* namespace HomeoneBoard */
