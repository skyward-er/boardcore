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

extern void defaultHandler(const mavlink_message_t* command);
extern void handlePingCommand(const mavlink_message_t* command);
extern void handleNoArgCommand(const mavlink_message_t* command);
extern void handleLaunchCommand(const mavlink_message_t* command);
extern void handleStatusRequestCommand(const mavlink_message_t* command);
extern void handleCalibrationCommand(const mavlink_message_t* command);
extern void handleRawEventMessage(const mavlink_message_t* command);

namespace HomeoneBoard
{
namespace TMTC
{

/*
 * Constructor: initialise objects (has memory allocation).
 */
TMTCManager::TMTCManager()
{
    gamma     = new Gamma868(RADIO_DEVICE_NAME);
    outBuffer = new CircularBuffer(TMTC_OUT_BUFFER_SIZE);
    // TODO: check gamma status and configuration
}

/*
 * Non-blocking send function: copies the message in the outBuffer if there's
 * enough space.
 */
bool TMTCManager::enqueueMsg(const uint8_t* msg, const uint8_t len)
{
    if (outBuffer->freeSize() >= len)
    {
        outBuffer->write(msg, len);
        return true;
    }

    return false;
}

/*
 * Sending thread's run() function: read from the outBuffer and forward on the
 * link.
 */
void TMTCManager::runSender()
{
    uint8_t msgTemp[TMTC_MAX_PKT_SIZE];

    while (1)
    {
        if (outBuffer->occupiedSize() > 0)
        {
            // Read from the buffer at maximum MAX_PKT_SIZE bytes
            uint32_t readBytes = outBuffer->read(msgTemp, TMTC_MAX_PKT_SIZE);

            // Try sending the packet (multiple times)
            for (int i = 0; i < TMTC_MAX_TRIES_PER_PACKET; i++)
            {
                bool sent = gamma->send(msgTemp, readBytes);
                if (sent)
                    break;
            }
        }
        miosix::Thread::sleep(TMTC_SEND_TIMEOUT);
    }
}

/*
 * Receiving thread's run() function: parse the received packet one byte at a
 * time
 * until you find a complete mavlink message and dispatch it with the
 * appropriate handler.
 */
void TMTCManager::runReceiver()
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t byte;

    while (1)
    {
        gamma->receive(&byte, 1);  // Blocking function

        // Parse one char at a time until you find a complete message
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status))
        {
#ifdef DEBUG
            printf(
                "Received message with ID %d, sequence: %d from component %d "
                "of system %d\n",
                msg.msgid, msg.seq, msg.compid, msg.sysid);
#endif

            // If the received message is not a HEARTBEAT, send an ACK back to
            // ground
            if (msg.msgid != MAVLINK_MSG_ID_HEART_BEAT)
                sendAck(&msg);

            // Handle the command
            if (msgHandlersMap.find(msg.msgid) != msgHandlersMap.end())
            {
                MessageHandler_t handler = msgHandlersMap[msg.msgid];
                (*handler)(&msg);
            }
            else
            {
                TMTC_TRACE("Unhandled message.");
            }
        }

        // TODO: aggiornare statistiche TMTC nell'housekeeping
        // sBoard->updateTMTCStats(&status);
    }
}

/*
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
    printf("Sent Ack: ");
    for (int i = 0; i < msgLen; i++)
        printf("%x ", bufferMsg[i]);
    printf("\n");
#endif

    if (!ackSent)
    {
        // TODO: fault counter? retry?
    }
}
}
}
