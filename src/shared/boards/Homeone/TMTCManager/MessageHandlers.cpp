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

#include "MessageHandlers.h"

namespace HomeoneBoard
{
namespace TMTC
{

/**
 * Check that the map contains the given msgId and return it, else return the
 * defaultHandler.
 */
MessageHandler_t getMsgHandler(const uint8_t msgId)
{
    if (msgHandlersMap.find(msgId) != msgHandlersMap.end())
    {
        return msgHandlersMap[msgId];
    }
    else
    {
        return (MessageHandler_t)&defaultHandler;
    }
}

/**
 * Default message handler: print a message.
 */
void defaultHandler(const mavlink_message_t* command)
{
    TMTC_TRACE("Received unknown message.\n");
}

/**
 * Handle Ping command: post the event on the EventBroker.
 */
void handlePingCommand(const mavlink_message_t* command)
{
    generatedEvt.sig = EV_PING_RECEIVED;
    sEventBroker->post(generatedEvt, DIAGNOSTICS);
}

/**
 * Handle a no argument command according to command id.
 */
void handleNoArgCommand(const mavlink_message_t* command)
{
    // Retrieve the command id from the payload of the message.
    uint8_t msgId = mavlink_msg_noarg_tc_get_command_id(command);

    // Handle according to the id
    switch (msgId)
    {
        // If the request is for debug info, get them directly from the Board
        // object.
        case MAV_CMD_REQ_DEBUG_INFO:
        {
            // TODO recuperare info direttamente da sBoard
            break;
        }
        // In any other case, post an event according to the translation map.
        default:
        {
            if (noArgCmdMap.find(msgId) != noArgCmdMap.end())
            {
                eventEntry_t evtEntry = noArgCmdMap[msgId];
                generatedEvt.sig      = evtEntry.event;
                sEventBroker->post(generatedEvt, evtEntry.topicID);
            }
            else
            {
                TMTC_TRACE("Received unknown noarg command message.\n");
            }

            break;
        }
    }
}

/**
 * Handle the Launch Command.
 */
void handleLaunchCommand(const mavlink_message_t* command)
{
    generatedEvt.sig = EV_START_LAUNCH;
    sEventBroker->post(generatedEvt, FLIGHT_EVENTS);
    // TODO: use the real launch event.
}

/**
 * Handle a Status request depending on the payload of the message.
 */
void handleStatusRequestCommand(const mavlink_message_t* command)
{
    // Handle depending the board id
    switch (mavlink_msg_request_board_status_tc_get_board_id(command))
    {
        case MAV_HOMEONE_BOARD:
        {
            // TODO: recuperare direttamente da sBoard.
            break;
        }
        case MAV_IGNITION_BOARD:
        {
            generatedEvt.sig = EV_IGNITION_STATUS_REQUEST;
            sEventBroker->post(generatedEvt, DIAGNOSTICS);
            break;
        }
        case MAV_NOSECONE_BOARD:
        {
            generatedEvt.sig = EV_NOSECONE_STATUS_REQUEST;
            sEventBroker->post(generatedEvt, DIAGNOSTICS);
            break;
        }
        case MAV_ALL_BOARDS:
        {
            // TODO
            break;
        }
    }
}

/**
 * Handle the calibration command: post the corresponding event in the
 * eventBroker.
 */
void handleCalibrationCommand(const mavlink_message_t* command)
{
    // TODO
}

/**
 * Handle a raw_event message: post the event contained in the payload
 * of the message directly in the EventBroker.
 * This message can only be received during test phase, and SHALL NOT be
 * part of the launch software.
 */
void handleRawEventMessage(const mavlink_message_t* command)
{
#ifdef DEBUG
    uint8_t evId     = mavlink_msg_raw_event_tc_get_Event_id(command);
    uint8_t topicId  = mavlink_msg_raw_event_tc_get_Topic_id(command);
    generatedEvt.sig = evId;
    sEventBroker->post(generatedEvt, topicId);
#endif
}
}
}
