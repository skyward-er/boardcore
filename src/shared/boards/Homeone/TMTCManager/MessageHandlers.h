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
#ifndef TMTC_MESSAGE_HANDLERS_H
#define TMTC_MESSAGE_HANDLERS_H

#include "TMTC_Config.h"

namespace HomeoneBoard
{
namespace TMTC
{

/* Define a standard function prototype for handling mavlink messages. */
typedef void (*MessageHandler_t)(const mavlink_message_t* command);

/* Define the couple <EVENTID, TOPICID> to use it in the noargCmdMap */
typedef struct eventEntry_type
{
    Events event;
    Topics topicID;
} eventEntry_t;

/*
 * MessageHandlers: functions that specify how a message of a certain type
 * should be handled.
 */
void defaultHandler(const mavlink_message_t* command);
void handlePingCommand(const mavlink_message_t* command);
void handleNoArgCommand(const mavlink_message_t* command);
void handleLaunchCommand(const mavlink_message_t* command);
void handleStatusRequestCommand(const mavlink_message_t* command);
void handleCalibrationCommand(const mavlink_message_t* command);
void handleRawEventMessage(const mavlink_message_t* command);

Event generatedEvt;

// clang-format off
/* Map that contains a handler reference for each handled command. */
std::map<uint8_t, MessageHandler_t> msgHandlersMap = {
    {
        MAVLINK_MSG_ID_PING_TC,
        (MessageHandler_t)&handlePingCommand
    },
    {
        MAVLINK_MSG_ID_NOARG_TC,
        (MessageHandler_t)&handleNoArgCommand
    },
    {
        MAVLINK_MSG_ID_START_LAUNCH_TC,
        (MessageHandler_t)&handleLaunchCommand
    },
    {
        MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC,
        (MessageHandler_t)&handleStatusRequestCommand
    },
    {
        MAVLINK_MSG_ID_CALIBRATE_BAROMETERS_TC,
        (MessageHandler_t)&handleCalibrationCommand
    },
    {
        MAVLINK_MSG_ID_RAW_EVENT_TC,
        (MessageHandler_t)&handleRawEventMessage
    }
};

/*
 * Map that contains an <EventID,TopicID> couple for each noarg_command that can
 * be directly mapped to an event.
 */
std::map<uint8_t, eventEntry_t> noArgCmdMap = {
    {
        MAV_CMD_ARM,
        {.event = EV_ARM, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_DISARM,
        {.event = EV_DISARM, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_ABORT,
        {.event = EV_IGNITION_STATUS_REQUEST, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_NOSECONE_OPEN,
        {.event = EV_NOSECONE_OPEN, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_NOSECONE_CLOSE,
        {.event = EV_NOSECONE_CLOSE, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_START_SAMPLING,
        {.event = EV_START_SAMPLING, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_STOP_SAMPLING,
        {.event = EV_STOP_SAMPLING, .topicID = FLIGHT_EVENTS}
    },
    {
        MAV_CMD_TEST_MODE,
        {.event = EV_TEST_MODE, .topicID = STATE_MACHINE}
    },
    {
        MAV_CMD_BOARD_RESET,
        {.event = EV_RESET_BOARD, .topicID = STATE_MACHINE}
    }
};
// clang-format on

/*
 * Getter for the Handler function of a Mavlink message.
 * @param msgId   the id of the Mavlink message (i.e. the code which identifies
 *                the message type).
 * @return        the function that handles that message as defined in the
 *                msgHandlersMap
 *                (or the defaultHandler() if the command is not directly
 *                handled).
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

}
}

#endif /* TMTC_MESSAGE_HANDLERS_H */
