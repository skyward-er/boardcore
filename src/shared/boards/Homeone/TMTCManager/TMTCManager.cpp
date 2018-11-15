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

#define TC(x) MAVLINK_MSG_ID_##X##_TC

namespace HomeoneBoard
{
namespace TMTC
{

/**
 * Constructor: initialise objects (has memory allocation).
 */
TMTCManager::TMTCManager()
{
    device   = new Gamma868("/dev/radio");
    sender   = new MavSender(device);
    receiver = new MavReceiver(device, &handleMavlinkMessage);

    sender->start();
    receiver->start()

    TRACE("%s", "[TMTC] Created TMTCManager\n");
}


/**
 * Send an ACK to notify the sender that you received the given message.
 */
void TMTCManager::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg,
                                    msg.msgid, msg.seq);

    /* Send the message back to the sender */
    send(ackMsg);
    TRACE("[TMTC] Enqueued Ack\n", 0);
}


/**
 *  Handle the Mavlink message, posting the corresponding event if needed.
 */
void TMTCManager::handleMavlinkMessage(const mavlink_message_t& msg)
{
    uint8_t msgId = msg.msgid;

    // TODO: reschedule GS_OFFLINE event

    switch (msgId)
    {
        case TC(NOARG):
        {
            uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(&msg);

            try
            {
                uint8_t sig = noargCmdToEvt.at(commandId);
                sEventBroker->post(Event {sig}, TOPIC_COMMANDS);
            } 
            catch (int e) 
            {
                TRACE("[TMTC] Unkown noArg command %d\n", commandId);
            }

            break;
        }

        case TC(REQUEST_BOARD_STATUS):
        {
            uint8_t boardId = mavlink_msg_request_board_status_tc_get_board_id(msg);

            try
            {
                uint8_t sig = statusCmdToEvt.at(boardId);
                sEventBroker->post(Event {sig}, TOPIC_COMMANDS);
            } 
            catch (int e) 
            {
                TRACE("[TMTC] Unkown status command %d\n", boardId);
            }
           
            break;
        }

        case TC(PING):
        {
            sEventBroker->post(Event {EV_PING_RECEIVED}, TOPIC_COMMANDS);
            break;
        }

        case TC(START_LAUNCH):
        {
            StartLaunchEvent startLaunchEvt;
            startLaunchEvt.sig = EV_TC_START_LAUNCH;
            startLaunchEvt.launchCode = mavlink_msg_start_launch_tc_get_launch_code(msg);

            sEventBroker->post(startLaunchEvt, TOPIC_COMMANDS);
            break;
        }

        case TC(CALIBRATE_BAROMETERS):
        {
            AltimeterCalibrationEvent generatedCalibEvt;
            generatedCalibEvt.sig = EV_TC_ALTIMETER_CALIBRATION;
            generatedCalibEvt.T0 = mavlink_msg_calibrate_barometers_tc_get_T0(msg);
            generatedCalibEvt.P0 = mavlink_msg_calibrate_barometers_tc_get_P0(msg);

            sEventBroker->post(generatedCalibEvt, TOPIC_COMMANDS);
            break;
        }

        case TC(RAW_EVENT):
        {
#ifdef DEBUG
            /* Retrieve event from the message*/
            Event evt = {mavlink_msg_raw_event_tc_get_Event_id(msg)};
            sEventBroker->post(evt, mavlink_msg_raw_event_tc_get_Topic_id(msg));
#endif
            break;
        }

        default:
        {
            TRACE("%s", "[TMTC] Received message is not of a known type\n");
            status.healthStatus = COMP_ERROR;
            // TODO: fault counter?
            break;
        }
    }
}

} /* namespace HomeoneBoard */
} /* namespace TMTC */
