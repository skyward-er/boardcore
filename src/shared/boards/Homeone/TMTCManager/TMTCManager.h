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

#ifndef TMTCMANAGER_H
#define TMTCMANAGER_H

#include "TMTCStatus.h"

#include "events/FSM.h"
#include "boards/Homeone/Events.h"

#include <drivers/gamma868/Gamma868.h>
#include <drivers/mavlink/MavSender.h>
#include <drivers/mavlink/MavReceiver.h>

namespace HomeoneBoard
{
namespace TMTC
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
        sender->enqueueMsg(msg);
    }

protected:
private:
    Gamma868* device;
    MavSender* sender;
    MavReceiver* receiver;

    HomeoneBoard::TMTC::TMTCStatus status;

    /* State handlers */
    void stateIdle(const Event& ev);
    void stateHighRateTM(const Event& ev);
    void stateLowRateTM(const Event& ev);

    // Maximum number of messages in the queue
    static const unsigned int TMTC_OUT_BUFFER_SIZE = 1000;
    // Minimum sleep time between sends
    static const unsigned int TMTC_MIN_GUARANTEED_SLEEP = 250;
    // Maximum number of consecutive messages sent before sleeping
    static const unsigned int TMTC_MAX_PKT_SIZE = 1*sizeof(mavlink_message_t);

    static const unsigned int LR_TM_TIMEOUT = 1000;
    static const unsigned int HR_TM_TIMEOUT = 250;
};

/**
 * Map each noArg command to the corresponding event 
 */
static const std::map<uint8_t, uint8_t> noargCmdToEvt = 
{
    { MAV_CMD_ARM,              EV_TC_ARM    }, 
    { MAV_CMD_DISARM,           EV_TC_DISARM }, 
    { MAV_CMD_ABORT,            EV_TC_ABORT_LAUNCH  }, 
    { MAV_CMD_NOSECONE_OPEN,    EV_TC_NC_OPEN }, 
    { MAV_CMD_NOSECONE_CLOSE,   EV_TC_NC_CLOSE }, 
    { MAV_CMD_START_LOGGING,    EV_TC_START_LOGGING }, 
    { MAV_CMD_STOP_LOGGING,     EV_TC_STOP_LOGGING }, 
    { MAV_CMD_TEST_MODE,        EV_TC_TEST_MODE   }, 
    { MAV_CMD_BOARD_RESET,      EV_TC_BOARD_RESET }, 
    { MAV_CMD_MANUAL_MODE,      EV_TC_MANUAL_MODE },
    { MAV_CMD_CUT_ALL,          EV_TC_CUT_ALL },
    { MAV_CMD_CUT_FIRST_DROGUE, EV_TC_CUT_FIRST_DROGUE },
    { MAV_CMD_END_MISSION,      EV_TC_END_MISSION }
};

} /* namespace HomeoneBoard */
} /* namespace TMTC */

#endif /* TMTCMANAGER_H */
