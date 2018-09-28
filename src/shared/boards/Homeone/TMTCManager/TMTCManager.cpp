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
    outBuffer = new CircularBuffer(TMTC_OUT_BUFFER_SIZE);

    receiverThread = miosix::Thread::create(receiverLauncher, TMTC_RECEIVER_STACKSIZE, TMTC_RECEIVER_PRIORITY,
                                           reinterpret_cast<void*>(this));
    senderThread = miosix::Thread::create(senderLauncher, TMTC_SENDER_STACKSIZE, TMTC_SENDER_PRIORITY,
                                           reinterpret_cast<void*>(this));

    printf("[TMTC] Created TMTCManager with a %d bytes buffer.\n", TMTC_OUT_BUFFER_SIZE);
    
    // TODO: check gamma status and configuration

    status.healthStatus = COMP_OK;
}

/**
 * Non-blocking send function: copies the message in the outBuffer if there's
 * enough space.
 */
bool TMTCManager::enqueueMsg(const uint8_t* msg, const uint8_t len)
{
    if (outBuffer->freeSize() >= len)
    {
        outBuffer->write(msg, len);

        printf("[TMTC] Enqueueing\n");
        // outBuffer->printContent();

        return true;
    }

    return false;
}

/**
 * Sending thread's run() function: read from the outBuffer and forward on the
 * link.
 */
void TMTCManager::runSender()
{
    uint8_t msgTemp[TMTC_MAX_PKT_SIZE];

    printf("[TMTC] Sender is running\n");
    // outBuffer->printContent();

    while (1)
    {
        if (outBuffer->occupiedSize() > 0)
        {

            printf("[TMTC] Sender is sending\n");
            // outBuffer->printContent();

            // Read from the buffer at maximum MAX_PKT_SIZE bytes
            uint32_t readBytes = outBuffer->read(msgTemp, TMTC_MAX_PKT_SIZE);

            bool sent = gamma->send(msgTemp, readBytes);
            if (sent)
                break;
            else
                status.sendErrors++;

            printf("[TMTC] Sent message\n");
        }
        // TODO: should this be done only if something has been sent?
        miosix::Thread::sleep(TMTC_SEND_TIMEOUT);
    }


    status.healthStatus = COMP_FAILED;
}

/**
 * Receiving thread's run() function: parse the received packet one byte at a
 * time until you find a complete Mavlink message and dispatch it.
 */
void TMTCManager::runReceiver()
{
    mavlink_message_t msg;
    uint8_t byte;

    while (1)
    {
        gamma->receive(&byte, 1);  // Blocking function

        // Parse one char at a time until you find a complete message
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &(status.mavstatus)))
        {

            printf(
                "[TMTC] Received message with ID %d, sequence: %d from "
                "component %d "
                "of system %d\n",
                msg.msgid, msg.seq, msg.compid, msg.sysid);

            // Send Ack
            if (msg.msgid != MAVLINK_MSG_ID_ACK_TM)
                sendAck(&msg);

            // Handle the command
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

            Event evt = {commandId}; // TODO: check that mavlink commands are same as events
            sEventBroker->post(evt, TOPIC_COMMANDS);
            break;
        }

        case MAVLINK_MSG_ID_REQUEST_BOARD_STATUS_TC:
        {
            uint8_t boardId = mavlink_msg_request_board_status_tc_get_board_id(msg);

            Event evt = {(uint8_t)(EV_NOSECONE_STATUS_REQUEST + boardId)}; // TODO: check that mavlink board order is the same as events
            sEventBroker->post(evt, TOPIC_COMMANDS);
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
            printf("[TMTC] Received message is not of a known type\n");
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

    // Create ack message passing the parameters
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg,
                            msg->msgid, msg->seq);
    // Convert it into a byte stream
    int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &ackMsg);

    // Send the message back to the sender
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
