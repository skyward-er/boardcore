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

#pragma once

#include "TMTCStatus.h"

#include "events/FSM.h"
#include "boards/Homeone/Events.h"

#include <drivers/gamma868/Gamma868.h>
#include <drivers/mavlink/MavManager.h>

namespace HomeoneBoard
{

// Mavlink messages sysID and compID
static const unsigned int TMTC_MAV_SYSID    = 1;
static const unsigned int TMTC_MAV_COMPID   = 1;

/**
 * The TMTCManager class handles the communication with the Ground Station.
 * It uses a Gamma868 transceiver and implements the Mavlink protocol.
 */
class TMTCManager : public FSM<TMTCManager>
{
public:
    TMTCManager();
    ~TMTCManager();

    /**
     * Non-blocking send wrapper.
     */
    void send(mavlink_message_t& msg)
    {
        mavManager->getSender(0)->enqueueMsg(msg);
    }

protected:
private:
    Gamma868* device;
    MavManager* mavManager;

    HomeoneBoard::TMTCStatus status;

    /* State handlers */
    void stateIdle(const Event& ev);
    void stateHighRateTM(const Event& ev);
    void stateLowRateTM(const Event& ev);

    /* Minimum sleep time between sends */
    static const unsigned int TMTC_MIN_GUARANTEED_SLEEP = 250;

    static const unsigned int LR_TM_TIMEOUT = 1000;
    static const unsigned int HR_TM_TIMEOUT = 250;
};

} /* namespace HomeoneBoard */
