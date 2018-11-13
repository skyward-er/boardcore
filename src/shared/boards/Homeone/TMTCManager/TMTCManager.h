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

#include <Singleton.h>
#include <events/EventBroker.h>

#include <drivers/gamma868/Gamma868.h>

namespace HomeoneBoard
{
namespace TMTC
{

/**
 * The TMTCManager class handles the communication with the Ground Station.
 * It uses a Gamma868 transceiver and implements the Mavlink protocol.
 */
class TMTCManager : public Singleton<TMTCManager>
{
    friend class Singleton<TMTCManager>;

public:
    static const char* DEVICE_NAME = "/dev/radio";
    static const uint8_t TMTC_MAV_SYSID = 1;
    static const uint8_t TMTC_MAV_COMPID = 1;

    /**
     * Class destructor.
     */
    ~TMTCManager() {}

    /**
     * Non-blocking send wrapper.
     */
    void send(mavlink_message_t& msg)
    {
        sender->enqueueMsg(msg);
    }

protected:
private:

    /**
     * Private constructor that realizes the Singleton pattern.
     */
    TMTCManager();

    /**
     *  Handles the Mavlink message, posting the corresponding event if needed.
     * @param msg           mavlink message to handle
     */
    void handleMavlinkMessage(const mavlink_message_t& msg);

    /**
     * Sends an acknowledge message back to the Ground Station.
     * @param msg    Mavlink message to acknowledge.
     */
    void sendAck(const mavlink_message_t* msg);

    Gamma868* device;

    MavSender* sender;
    MavReceiver* receiver;

    TMTCStatus status;

    static const std::map<uint8_t, uint8_t> statusCmdToEvt = 
    {
        { MAV_NOSECONE_BOARD, EV_NOSECONE_STATUS_REQUEST },
        { MAV_IGNITION_BOARD, EV_IGNITION_STATUS_REQUEST },
        { MAV_HOMEONE_BOARD,  EV_HOMEONE_STATUS_REQUEST }
    };

    static const std::map<uint8_t, uint8_t> noargCmdToEvt = 
    {
        { MAV_CMD_ARM,              EV_TC_ARM    }, 
        { MAV_CMD_DISARM,           EV_TC_DISARM }, 
        { MAV_CMD_ABORT,            EV_TC_ABORT_LAUNCH  }, 
        { MAV_CMD_NOSECONE_OPEN,    EV_TC_NC_OPEN }, 
        { MAV_CMD_NOSECONE_CLOSE,   EV_TC_NC_CLOSE }, 
        { MAV_CMD_START_SAMPLING,   EV_TC_START_LOGGIN }, 
        { MAV_CMD_STOP_SAMPLING,    EV_TC_STOP_LOGGIN }, 
        { MAV_CMD_TEST_MODE,        EV_TC_TEST_MODE   }, 
        { MAV_CMD_BOARD_RESET,      EV_TC_BOARD_RESET }, 

        { MAV_CMD_MANUAL_MODE,      EV_TC_MANUAL_MODE },
        { MAV_CMD_CUT_ALL,          EV_TC_CUT_ALL },
        { MAV_CMD_CUT_FIRST_DROGUE, EV_TC_CUT_FIRST_DROGUE },
        { MAV_CMD_END_MISSION,      EV_TC_END_MISSION }
    };
};

} /* namespace HomeoneBoard */
} /* namespace TMTC */

/* Define a singleton object that can be accessed from other files */
#ifndef sTMTCManager
#define sTMTCManager HomeoneBoard::TMTC::TMTCManager::getInstance()
#else
#error TMTCMANAGER ALREADY DEFINED
#endif /* sTMTCManager */

#endif /* TMTCMANAGER_H */
