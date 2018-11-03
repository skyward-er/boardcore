/* Mavlink Sender
 *
 * Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#ifndef MAV_SENDER_H
#define MAV_SENDER_H

#include <Common.h>

#include <ActiveObject.h>
#include <drivers/Transceiver.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h> 

/**
 * Class to bufferize the sending of mavlink messages over a transceiver.
 */
class MavSender : public ActiveObject
{

public:
    static const uint16_t MAV_MIN_GUARANTEED_SLEEP = 500;
    static const uint16_t MAV_OUT_QUEUE_LEN = 10;

    /**
     * @param device   the device used for sending messages
     */
    MavSender(Transceiver* device) : ActiveObject(), device(device) {}
    
    ~MavSender(){};

    /**
     * Non-blocking send function
     */
    bool enqueueMsg(mavlink_message_t& msg)
    {
        miosix::Lock<miosix::FastMutex> l(mutex);

        if(messageQueue.isFull())
        {
            return false;
        }
        else {
            messageQueue.put(msg);
            TRACE("[MAV] Enqueueing %d bytes\n", msg->len);
            return true;
        }
    }

protected:
    Transceiver* device;

    /**
     * Ran in a separate thread. Periodically flushes the message queue
     * and sends all the enqueued messages.
     */
    void run() override
    {
        mavlink_message_t msgTemp;
        uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];
        TRACE("%s", "[MAV] Sender is running\n");

        while (1)
        {
            /* Read from the buffer and send */
            while (!(messageQueue.isEmpty()))
            {
                {
                    /* Get mavlink message from queue */
                    miosix::Lock<miosix::FastMutex> l(mutex);
                    messageQueue.get(msgTemp);
                }

                /* Convert into a byte stream */
                int msgLen = mavlink_msg_to_send_buffer(bufferMsg, &msgTemp);

                bool sent = device->send(bufferMsg, msgLen);
                TRACE("[MAV] Sending %lu bytes\n", msgLen);

                if (!sent)
                    TRACE("[MAV] Error: could not send message\n", 0);
            }

            /* Sleep guarantees that commands from the GS can be received */
            miosix::Thread::sleep(MAV_MIN_GUARANTEED_SLEEP);
        }
    }

private:
    miosix::Queue<mavlink_message_t, MAV_OUT_QUEUE_LEN> messageQueue;
    miosix::FastMutex mutex;
};

#endif /* MAV_SENDER_H */
