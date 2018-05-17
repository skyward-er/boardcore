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
#include "MessageHandler.h"

namespace HomeoneBoard
{
namespace TMTC
{

/**
 *  Command->Event translation map
 */
// clang-format off
std::map<msgEntry_t, evtEntry_t> MessageHandler::commandTranslationMap = {
	// No argument commands
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_ARM},
		{EV_TC_ARM,                 TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_DISARM},
		{EV_TC_DISARM,              TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_ABORT},
		{EV_ABORT_LAUNCH,           TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_NOSECONE_OPEN},
		{EV_TC_NOSECONE_OPEN,       TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_NOSECONE_CLOSE},
		{EV_TC_NOSECONE_CLOSE,      TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_START_SAMPLING},
		{EV_TC_START_SAMPLING,      TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_STOP_SAMPLING},
		{EV_TC_STOP_SAMPLING,       TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_TEST_MODE},
		{EV_TC_TEST_MODE,           TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_NOARG_TC,   MAV_CMD_BOARD_RESET},
		{EV_TC_RESET_BOARD,         TOPIC_COMMANDS}
	},
	// Board status request
	{
		{MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC, MAV_NOSECONE_BOARD},
		{EV_NOSECONE_STATUS_REQUEST,             TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC, MAV_IGNITION_BOARD},
		{EV_IGNITION_STATUS_REQUEST,             TOPIC_COMMANDS}
	},
	{
		{MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC, MAV_HOMEONE_BOARD},
		{EV_HOMEONE_STATUS_REQUEST,              TOPIC_COMMANDS}
	},
	// Ping message
	{
		{MAVLINK_MSG_ID_PING_TC,                 0},
		{EV_PING_RECEIVED,          TOPIC_COMMANDS}
	},
	// Start Launch command
	{
		{MAVLINK_MSG_ID_START_LAUNCH_TC,         0},
		{EV_TC_START_LAUNCH,        TOPIC_COMMANDS}
	},
	// Barometers calibration command
	{
		{MAVLINK_MSG_ID_CALIBRATE_BAROMETERS_TC,            0},
		{EV_TC_ALTIMETER_CALIBRATION,          TOPIC_COMMANDS}
	}
};
// clang-format on

/**
 *  Handle the Mavlink message, posting the corresponding event if needed
 */
void MessageHandler::handleMavlinkMessage(const mavlink_message_t* msg)
{
	uint8_t msgId = msg->msgid;

	msgEntry_t key;
	evtEntry_t evtEntry;

	switch (msgId)
	{
		case MAVLINK_MSG_ID_NOARG_TC:
		{
			/* Construct key */
			key = {msgId, mavlink_msg_noarg_tc_get_command_id(msg)};
			/* Retrieve event from map and post it */
			if(retrieveEvtEntry(key, &evtEntry)) {
				Event evt = {evtEntry.event};
				sEventBroker->post( evt, evtEntry.topicID );
			}
			break;
		}
		case MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC:
		{
			/* Construct key */
			key = {msgId,
						mavlink_msg_request_board_status_tc_get_board_id(msg)};
			/* Retrieve event from map and post it */
			if(retrieveEvtEntry(key, &evtEntry)) {
				Event evt = {evtEntry.event};
				sEventBroker->post( evt, evtEntry.topicID );
			}
			break;
		}
		case MAVLINK_MSG_ID_PING_TC:
		{
			/* Construct key */
			key = {msgId, 0};
			/* Retrieve event from map and post it */
			if(retrieveEvtEntry(key, &evtEntry)) {
				Event evt = {evtEntry.event};
				sEventBroker->post( evt, evtEntry.topicID );
			}
			break;
		}
		case MAVLINK_MSG_ID_START_LAUNCH_TC:
		{
			/* Construct key */
			key = {msgId, 0};
			/* Retrieve event from map and post it */
			if(retrieveEvtEntry(key, &evtEntry)) {
				StartLaunchEvent startLaunchEvt;
				startLaunchEvt.sig = evtEntry.event;
				startLaunchEvt.launchCode =
						mavlink_msg_start_launch_tc_get_launch_code(msg);

				sEventBroker->post( startLaunchEvt, evtEntry.topicID );
			}
			break;
		}
		case MAVLINK_MSG_ID_CALIBRATE_BAROMETERS_TC:
		{
			/* Construct key */
			key = {msgId, 0};
			/* Build event and post it */
			if(retrieveEvtEntry(key, &evtEntry)) {
				AltimeterCalibrationEvent generatedCalibEvt;
				generatedCalibEvt.sig = evtEntry.event;
				generatedCalibEvt.T0 =
							mavlink_msg_calibrate_barometers_tc_get_T0(msg);
				generatedCalibEvt.P0 =
							mavlink_msg_calibrate_barometers_tc_get_P0(msg);

				sEventBroker->post( generatedCalibEvt, evtEntry.topicID );
			}
			break;
		}
		case MAVLINK_MSG_ID_RAW_EVENT_TC:
		{
#ifdef DEBUG
			/* Retrieve event from the message*/
			Event evt = {mavlink_msg_raw_event_tc_get_Event_id(msg)};
			sEventBroker->post( evt, mavlink_msg_raw_event_tc_get_Topic_id(msg));
#endif
			break;
		}
	}
}

/**
 * Search the key in the map and put its value in the given pointer.
 */
bool MessageHandler::retrieveEvtEntry (msgEntry_t key, evtEntry_t* retrievedEntry)
{
    if (commandTranslationMap.find(key) != commandTranslationMap.end())
    {
        *retrievedEntry = commandTranslationMap.find(key)->second;
        return true;
    }
    else {
#ifdef DEBUG
        printf("[TMTC] Received unknown command message.\n");
#endif
        return false;
    }
}


}
}
