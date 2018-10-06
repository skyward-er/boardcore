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

namespace HomeoneBoard
{
namespace TMTC
{

/**
 * Constructor: initialise objects (has memory allocation).
 */
TMTCManager::TMTCManager()
{
    gamma     = new Gamma868(RADIO_DEVICE_NAME);
    outBuffer = new ByteSyncedCircularBuffer(TMTC_OUT_BUFFER_SIZE);

    receiverThread = miosix::Thread::create(
        receiverLauncher, TMTC_RECEIVER_STACKSIZE, TMTC_RECEIVER_PRIORITY,
        reinterpret_cast<void*>(this));
    senderThread = miosix::Thread::create(
        senderLauncher, TMTC_SENDER_STACKSIZE, TMTC_SENDER_PRIORITY,
        reinterpret_cast<void*>(this));

    TRACE("%s", "[TMTC] Created TMTCManager\n");
    
    // TODO: check gamma status and configuration?

    status.healthStatus = COMP_OK;
}

/**
 * Non-blocking send function
 */
bool TMTCManager::enqueueMsg(const uint8_t* msg, const uint8_t len)
{
    size_t written = outBuffer->put(msg, len);

    TRACE("[TMTC] Enqueueing %d bytes\n", len);

    return (written > 0);
}

/**
 * Sending thread's run() function
 */
void TMTCManager::runSender()
{
    uint8_t msgTemp[TMTC_MAX_PKT_SIZE];

    TRACE("%s", "[TMTC] Sender is running\n");

    while (1)
    {
        /* Read from the buffer at maximum MAX_PKT_SIZE bytes and send them */
        while ( !(outBuffer->isEmpty()) )
        {
            TRACE("[TMTC] Found %u bytes\n", outBuffer->count());
            uint32_t readBytes = outBuffer->pop(msgTemp, TMTC_MAX_PKT_SIZE);
            bool sent = gamma->send(msgTemp, readBytes);

            TRACE("[TMTC] Sending %lu bytes\n", readBytes);

            if (!sent)
                status.sendErrors++; // TODO: fault counter?
        }

        /* Sleep guarantees that commands from the GS can be received */
        miosix::Thread::sleep(TMTC_MIN_GUARANTEED_SLEEP);
    }

    status.healthStatus = COMP_FAILED;
}

/**
 * Receiving thread's run() function
 */
void TMTCManager::runReceiver()
{
    mavlink_message_t msg;
    uint8_t byte;

    while (1)
    {
        gamma->receive(&byte, 1);  // Blocking function

        /* Parse one char at a time until you find a complete message */
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &(status.mavstatus)))
        {
            TRACE(
                "[TMTC] Received message with ID %d, sequence: %d from "
                "component %d of system %d\n",
                msg.msgid, msg.seq, msg.compid, msg.sysid);

            /* Send Ack */
            if (msg.msgid != MAVLINK_MSG_ID_ACK_TM)
                sendAck(&msg);

            /* Handle the command */
            handleMavlinkMessage(&msg);
        }
    }

    status.healthStatus = COMP_FAILED;
}

/**
 *  Handle the Mavlink message, posting the corresponding event if needed.
 */
void TMTCManager::handleMavlinkMessage(const mavlink_message_t* msg)
{
    uint8_t msgId = msg->msgid;

    switch (msgId)
    {
        case MAVLINK_MSG_ID_NOARG_TC:
        {
            uint8_t commandId = mavlink_msg_noarg_tc_get_command_id(msg);

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

        case MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC:
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

        case MAVLINK_MSG_ID_PING_TC:
        {
            sEventBroker->post(Event {EV_PING_RECEIVED}, TOPIC_COMMANDS);
            break;
        }

        case MAVLINK_MSG_ID_START_LAUNCH_TC:
        {
            StartLaunchEvent startLaunchEvt;
            startLaunchEvt.sig = EV_TC_START_LAUNCH;
            startLaunchEvt.launchCode = mavlink_msg_start_launch_tc_get_launch_code(msg);

            sEventBroker->post(startLaunchEvt, TOPIC_COMMANDS);
            break;
        }

        case MAVLINK_MSG_ID_CALIBRATE_BAROMETERS_TC:
        {
            AltimeterCalibrationEvent generatedCalibEvt;
            generatedCalibEvt.sig = EV_TC_ALTIMETER_CALIBRATION;
            generatedCalibEvt.T0 = mavlink_msg_calibrate_barometers_tc_get_T0(msg);
            generatedCalibEvt.P0 = mavlink_msg_calibrate_barometers_tc_get_P0(msg);

            sEventBroker->post(generatedCalibEvt, TOPIC_COMMANDS);
            break;
        }

        case MAVLINK_MSG_ID_RAW_EVENT_TC:
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

/**
 * Send an ACK to notify the sender that you received the given message.
 */
void TMTCManager::sendAck(const mavlink_message_t* msg)
{
    uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];  // TODO Check this number
    mavlink_message_t ackMsg;

    /* Create ack message passing the parameters */
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg,
                            msg->msgid, msg->seq);
    /* Convert it into a byte stream */
    int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &ackMsg);

    /* Send the message back to the sender */
    bool ackSent = enqueueMsg(bufferMsg, msgLen);

#ifdef DEBUG
    printf("[TMTC] Sent Ack: ");
    for (int i = 0; i < msgLen; i++)
        printf("%x ", bufferMsg[i]);
    printf("\n");
#endif

    if (!ackSent)
    {
        status.ackErrors++;
    }
}

} /* namespace HomeoneBoard */
} /* namespace TMTC */
