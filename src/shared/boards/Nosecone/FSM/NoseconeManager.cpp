/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani, Alvise de' Faveri Tron
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

#include "NoseconeManager.h"
#include <events/EventBroker.h>

#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

using namespace miosix;

namespace NoseconeBoard
{

NoseconeManager::NoseconeManager(MotorDriver& motor) : 
            FSM(&NoseconeManager::state_idle), 
            motor(motor)
{
    /* Set intial status */
    status.close_received = false;
    status.open_received = false;
    log();
}

/**
 * From idle you can either open or close, in any order you desire.
 */
void NoseconeManager::state_idle(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            TRACE("IDLE state entry\n");
            motor.stop();
            break;

        case EV_EXIT:
            TRACE("IDLE state exit\n");
            break;

        case EV_OPEN:
            TRACE("IDLE state received EV_OPEN\n");
            transition(&NoseconeManager::state_opening);
            break;

        case EV_CLOSE:
            TRACE("IDLE state received EV_CLOSE\n");
            transition(&NoseconeManager::state_closing);
            break;

        default:
            TRACE("Unknown event received.\n");
            break;
    }
}

/**
 * From opening you can only go back to idle. This can be caused by:
 * - motor limit reached (nominal case)
 * - manual stop command
 * - timeout (worst case)
 * The corresponding flag is raised in all three cases, and reset on state entry.
 */
void NoseconeManager::state_opening(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            TRACE("[OPENING] entering\n");
            motor.start(MotorDirection::NORMAL_DIRECTION, OPENING_DUTY_CYCLE);
            delayedId = sEventBroker->postDelayed(Event{EV_OPEN_TIMER_EXPIRED},
                                                        TOPIC_NOSECONE, 
                                                        OPEN_TIMEOUT);

            /* Clear status bits */
            status.open_timeout = false;
            status.open_stop = false;
            status.open_limit = false;
            /* Set open received bit */
            status.open_received = true;
            log();
            break;

        case EV_EXIT:
            sEventBroker->removeDelayed(delayedId);
            TRACE("[OPENING] exiting\n");
            break;
        
        case EV_STOP:
            TRACE("[OPENING] received EV_STOP\n");
            status.open_stop = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        case EV_OPEN_TIMER_EXPIRED:
            TRACE("[OPENING] received EV_TIMER_EXPIRED\n");
            status.open_timeout = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        case EV_R_MOTOR_LIMIT:
            TRACE("[OPENING] received EV_MOTOR_LIMIT\n");
            status.open_limit = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        default:
            TRACE("Unknown event received.\n");
            break;
    }
}

/**
 * From closing you can only go back to idle. This can be caused by:
 * - motor limit reached (nominal case)
 * - manual stop command
 * - timeout (worst case)
 * The corresponding flag is raised in all three cases, and reset on state entry.
 */
void NoseconeManager::state_closing(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            TRACE("[CLOSING] entering\n");
            motor.start(MotorDirection::REVERSE_DIRECTION, CLOSING_DUTY_CYCLE);
            delayedId = sEventBroker->postDelayed(Event{EV_CLOSE_TIMER_EXPIRED}, 
                                                    TOPIC_NOSECONE, 
                                                    CLOSE_TIMEOUT);

            /* Clear status bits */
            status.close_timeout = false;
            status.close_stop = false;
            status.close_limit = false;
            /* Set close received bit */
            status.close_received = true;
            log();
            break;

        case EV_EXIT:
            sEventBroker->removeDelayed(delayedId);
            TRACE("[CLOSING] exiting\n");
            break;
        
        case EV_STOP:
            TRACE("[CLOSING] received EV_STOP\n");
            status.close_stop = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        case EV_CLOSE_TIMER_EXPIRED:
            TRACE("[CLOSING] received EV_TIMER_EXPIRED\n");
            status.close_timeout = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        case EV_L_MOTOR_LIMIT:
            TRACE("[CLOSING] received EV_MOTOR_LIMIT\n");
            status.close_limit = true;
            log();
            transition(&NoseconeManager::state_idle);
            break;

        default:
            TRACE("Unknown event received.\n");
            break;
    }
}

} /* namespace NoseconeBoard */
