/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#pragma once

#ifndef MAVLINK_H
    #error Wrong include order: you should include a MAVLINK_H before using this
#endif

#include <Common.h>
#include <ActiveObject.h>
#include <drivers/Transceiver.h>

#include <libs/mavlink_skyward_lib/mavlink_lib/protocol.h>

/**
 * Class to bufferize the sending of mavlink messages over a transceiver.
 */
class MavSender : public ActiveObject
{

public:
    static const uint16_t MAV_OUT_QUEUE_LEN  = 10;

    /**
     * @param device    the device used for sending messages
     * @param sleepTime min amount of ms to sleep after every send (guaranteed silence)
     */
    MavSender(Transceiver* device, mavlink_status_t* status, uint16_t sleepTime = 0) : 
                ActiveObject(), device(device), status(status), sleep_after_send(sleepTime) {}

    ~MavSender(){};

    /**
     * Non-blocking send function. Message is put in the queue if not full.
     * @param msg    message to send (mavlink struct)
     * @return true if the message could be enqueued (queue not full)
     */
    bool enqueueMsg(const mavlink_message_t& msg)
    {
        if (messageQueue.isFull())
        {
            return false;
        }
        else
        {
            messageQueue.put(msg);
            TRACE("[MAV] Enqueueing %d bytes\n", msg.len);
            return true;
        }
    }

protected:

    /**
     * Inherited by ActiveObject.
     * Periodically flushes the message queue and sends all the enqueued messages.
     * After every send, a sleep is done to guarantee some silence on the channel.
     */
    void run() override
    {
        mavlink_message_t msgTemp;
        uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];
        TRACE("%s", "[MAV] Sender is running\n");

        while (1)
        {
            /* Read mavlink message from buffer */
            messageQueue.waitUntilNotEmpty();
            messageQueue.get(msgTemp);

            /* Convert into a byte stream */
            int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &msgTemp);

            /* Send */
            bool sent = device->send(bufferMsg, msgLen);

            TRACE("[MAV] Sending %d bytes\n", msgLen);

            if (!sent)
                TRACE("[MAV] Error: could not send message\n");

            /* Update status */
            status->current_tx_seq++;

            /* Sleep guarantees that commands from the GS can be received */
            miosix::Thread::sleep(sleep_after_send);
        }
    }

private:
    Transceiver* device;
    mavlink_status_t* status;
    miosix::Queue<mavlink_message_t, MAV_OUT_QUEUE_LEN> messageQueue;

    const uint16_t sleep_after_send;
};