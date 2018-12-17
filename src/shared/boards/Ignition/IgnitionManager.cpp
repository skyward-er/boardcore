/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
#pragma once

#include "IgnitionManager.h"

namespace IgnBoard
{

using namespace actuators;

/**
 * Canbus receiving function.
 */
void canRcv(CanMsg message, IgnitionManager* mngr) 
{
    TRACE("[CAN] Received message with id %d\n", message.StdId);

    switch (message.StdId)
    {
        case CanInterfaces::CAN_TOPIC_HOMEONE:
            if(message.Data[0] == CanInterfaces::CAN_MSG_ABORT) {
                myStatus.u1_abort_cmd = 0b1;
                mngr->abort();
            } 
            else if(message.Data[0] == CanInterfaces::CAN_MSG_REQ_IGN_STATUS) {
                mngr->getStatus();
            } 
            else {
                TRACE("[CAN] Message not recognized\n");
            }
        break;

        case CanInterfaces::CAN_TOPIC_LAUNCH:
            uint64_t code = 0;
            memcpy(&code, message.Data, 8);

            mngr->launch(code);
        break;

        default:
            TRACE("[CAN] Message not recognized\n");
            break;
    }
}


/* Manager constructor: init canbus, set internal state and send state on canbus */
IgnitionManager::IgnitionManager()
{
    CanManager c = new CanManager(CAN1);
    initCanbus(c);

    // Communication with board 2?
    
    myStatus = 0;
    sendStatus();
}

/* Initialise CAN1 on PA11, PA12, set filters and set receiver function. */
void IgnitionManager::initCanbus(CanManager& c)
{
    /* Initialise canbus with hardware filters. */
    c.addHWFilter(CanInterfaces::CAN_TOPIC_HOMEONE, 0);
    c.addHWFilter(CanInterfaces::CAN_TOPIC_LAUNCH, 0);

    canbus_init_t can_conf = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};

    /* Transform canRcv(message, mngr) into canRcv(message) (see std::bind reference). */
    CanHandler handler = std::bind(canRcv, std::placeholders::_1, this);

    /* Add canbus and define pins, configuration and receiving function. */
    c.addBus<GPIOA_BASE, 11, 12>(can_conf, &handler);

    TRACE("[CAN] Initialised CAN1 on PA11-12 \n");
}


/* Ignition Functions */

void IgnitionManager::abort() 
{
    abortPin::low();
    Thread::sleep(ABORT_DURATION);
    abortPin::high();

    sendStatus();
}

void IgnitionManager::getStatus()
{
    // getStatus from other board
    // refresh myStatus
    sendStatus();
}

void IgnitionManager::launch(uint64_t launch_code)
{
    if(myStatus.u1_abort_cmd == 0)
    {
        if(checkLaunchCode(launch_code)) 
        {
            // send launch code to other board
            // poll for response
            // if response negativa
                // myStatus.u2_wrong_code = 0b1;
                // abort()
            // else if nCycle > 1000
                // myStatus.u1_abort_timeout = 0b1;
                // abort()
            // else
                ignitionPin::high();
                Thread::sleep(LAUNCH_DURATION);
                ignitionPin::low();
        } 
        else {
            myStatus.u1_wrong_code = 0b1;
            abort();
        }
    } 
    else {
        TRACE("Received launch while aborted\n");
    }  
}

void IgnitionManager::sendStatus() 
{
    canManager->getBus(0)->send(CanInterfaces::CAN_MSG_IGN_STATUS, 
                                    (uint8_t*)myStatus, sizeof(IgnitionBoardStatus));
}

bool IgnitionManager::checkLaunchCode(uint64_t launch_code) 
{
    return launch_code == EXPECTED_LAUNCH_CODE;
}

}
