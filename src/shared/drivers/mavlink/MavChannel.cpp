 /*
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#define COMPILING_CPP

#include <Common.h>
#include <drivers/Transceiver.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/protocol.h>
#include "MavStatus.h"
#include "MavChannel.h"

/**
 * @brief  Start the receiving and sending threads
 * @return false if one or both could not start
 */
bool MavChannel::start()
{
    if (!sndStarted)
    {
        // Start sender
        sndThread = miosix::Thread::create( sndLauncher, 
            miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::MAIN_PRIORITY
            reinterpret_cast<void*>(this), miosix::Thread::JOINABLE );

        if (sndThread != nullptr)
            sndStarted = true;
    }

    if (!rcvStarted)
    {
        // Start receiver
        rcvThread = miosix::Thread::create( rcvLauncher, 
            miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::MAIN_PRIORITY
            reinterpret_cast<void*>(this), miosix::Thread::JOINABLE );

        if (rcvThread != nullptr)
            rcvStarted = true;
    }

    return (sndStarted && rcvStarted);
}

/**
 * Reads one char at a time from the device and tries to parse a mavlink message.
 * If the message is successfully parsed, executes the function that was
 * passed in the constructor.
 */
void MavChannel::runReceiver()
{
    mavlink_message_t msg;
    uint8_t byte;

    while(true)
    {
        device->receive(&byte, 1);  // Blocking function

        {
            // parse next byte
            miosix::Lock<FastMutex> l(mtx_status);
            uint8_t parse_result = mavlink_parse_char(MAVLINK_COMM_0, byte, 
                                                        &msg, &(status.mav_stats))
        }

        /* If a complete mavlink message was found */
        if (parse_result)
        {
            TRACE("[MAV] Received message with ID %d, sequence: %d from "
                "component %d of system %d\n",
                msg.msgid, msg.seq, msg.compid, msg.sysid);

            /* Handle the command */
            handleMavlinkMessage(this, msg);
        }
    }
}

/**
 * Periodically flushes the message queue and sends all the enqueued messages.
 * After every send, a sleep is done to guarantee some silence on the channel.
 */
void MavChannel::runSender()
{
    mavlink_message_t msgTemp;
    uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];
    TRACE("%s", "[MAV] Sender is running\n");

    while(true)
    {
        /* Readfrom buffer */
        messageQueue.waitUntilNotEmpty();
        messageQueue.get(msgTemp);

        /* Update status */
        {
            miosix::Lock<FastMutex> l(mtx_status);
            status.n_send_queue--;
        }

        /* Convert into a byte stream */
        int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &msgTemp);

        /* Send */
        bool sent = device->send(bufferMsg, msgLen);
        TRACE("[MAV] Sending %d bytes\n", msgLen);

        if (!sent)
            TRACE("[MAV] Error: could not send message\n");

        /* Update status */
        {
            miosix::Lock<FastMutex> l(mtx_status);
            status.current_tx_seq++;
            if (!sent) status.n_send_errors++;
        }

        /* Sleep guarantees that commands from the GS can be received */
        miosix::Thread::sleep(sleep_after_send);
    }
}

/**
 * Non-blocking send function. Message is put in the queue if not full.
 * @param msg    message to send (mavlink struct)
 * @return true if the message could be enqueued (queue not full)
 */
bool MavChannel::enqueueMsg(const mavlink_message_t& msg)
{
    if (messageQueue.isFull())
    {
        /* Update status */
        {
            miosix::Lock<FastMutex> l(mtx_status);
            status.n_send_errors++;
        }

        return false;
    }
    
    else
    {
        /* Update status */
        {
            miosix::Lock<FastMutex> l(mtx_status);
            status.n_send_queue++;
            if (status.n_send_queue > status.max_send_queue)
                status.max_send_queue = status.n_send_queue
        }

        messageQueue.put(msg);
        TRACE("[MAV] Enqueueing %d bytes\n", msg.len);
        return true;
    }
}

/* Synchronized getter */
MavStatus MavChannel::getStatus()
{
    miosix::Lock<FastMutex> l(mtx_status);
    return status;
}