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

#pragma once

/* WARNING: due to this guard, moving implementation into a .cpp file won't compile */
#ifndef MAVLINK_H
    #error Wrong include order: you should include a MAVLINK_H before using this
#endif

#include <Common.h>
#include <drivers/Transceiver.h>
#include "MavStatus.h"

#define MAV_OUT_QUEUE_LEN 10

/**
 * Class to parse mavlink messages through a Transceiver. Lets you
 * choose the function to execute when a message is received.
 */
class MavChannel
{

/* Alias of the function to be executed on message rcv */
using MavHandler = std::function<void(MavChannel* channel, const mavlink_message_t& msg)>;

public:
    /**
     * @param device     device used to send and receive
     * @param onRcv      function to be executed on message rcv 
     * @param sleepTime  min sleep after send
     */
    MavChannel(Transceiver* device, MavHandler onRcv, uint16_t sleepTime = 0) : 
                    device(device), handleMavlinkMessage(onRcv), sleep_after_send(sleepTime) 
    {
        memset(&status, 0, sizeof(MavStatus));
    }

    ~MavChannel() {}

    /**
     * @brief  Start the receiving and sending threads
     * @return false if one or both could not start
     */
    bool start()
    {
        stop_flag = false;

        // Start sender
        if (!sndStarted)
        {
            sndThread = miosix::Thread::create( sndLauncher, 
                miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::MAIN_PRIORITY,
                reinterpret_cast<void*>(this), miosix::Thread::JOINABLE );

            if (sndThread != nullptr)
                sndStarted = true;
        }

        // Start receiver
        if (!rcvStarted)
        {
            rcvThread = miosix::Thread::create( rcvLauncher, 
                miosix::STACK_DEFAULT_FOR_PTHREAD, miosix::MAIN_PRIORITY,
                reinterpret_cast<void*>(this), miosix::Thread::JOINABLE );

            if (rcvThread != nullptr)
                rcvStarted = true;
        }

        return (sndStarted && rcvStarted);
    }

    /**
     * Stops sender and receiver.
     */
    void stop()
    {
        stop_flag = true;
        
        sndThread->join();
        rcvThread->join();
    }

    /**
     * Reads one char at a time from the device and tries to parse a mavlink message.
     * If the message is successfully parsed, executes the function that was
     * passed in the constructor.
     */
    void runReceiver()
    {
        mavlink_message_t msg;
        uint8_t byte;

        while(!stop_flag)
        {
            device->receive(&byte, 1);  // Blocking function
            uint8_t parse_result = 0;
            {
                // parse next byte
                miosix::Lock<miosix::FastMutex> l(mtx_status);
                parse_result = mavlink_parse_char(MAVLINK_COMM_0, byte, 
                                                            &msg, &(status.mav_stats));
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
    void runSender()
    {
        mavlink_message_t msgTemp;
        uint8_t bufferMsg[sizeof(mavlink_message_t) + 1];
        TRACE("%s", "[MAV] Sender is running\n");

        while(!stop_flag)
        {
            /* Readfrom buffer */
            messageQueue.waitUntilNotEmpty();
            messageQueue.get(msgTemp);

            /* Update status */
            {
                miosix::Lock<miosix::FastMutex> l(mtx_status);
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
                miosix::Lock<miosix::FastMutex> l(mtx_status);
                status.mav_stats.current_tx_seq++;
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
    bool enqueueMsg(const mavlink_message_t& msg)
    {
        if (messageQueue.isFull())
        {
            /* Update status */
            {
                miosix::Lock<miosix::FastMutex> l(mtx_status);
                status.n_send_errors++;
            }

            return false;
        }
        
        else
        {
            /* Update status */
            {
                miosix::Lock<miosix::FastMutex> l(mtx_status);
                status.n_send_queue++;
                if (status.n_send_queue > status.max_send_queue)
                    status.max_send_queue = status.n_send_queue;
            }

            messageQueue.put(msg);
            TRACE("[MAV] Enqueueing %d bytes\n", msg.len);
            return true;
        }
    }

    /* Synchronized getter */
    MavStatus getStatus() 
    {
        miosix::Lock<miosix::FastMutex> l(mtx_status);
        return status;
    }

    /* Setter */
    void setSleepAfterSend(uint16_t newSleepTime)
    {
        sleep_after_send = newSleepTime;
    }

private:
    /**
     * Calls the run member function
     * \param arg the object pointer cast to void*
     */
    static void rcvLauncher(void* arg)
    {
        reinterpret_cast<MavChannel*>(arg)->runReceiver();
    }

    /**
     * Calls the run member function
     * \param arg the object pointer cast to void*
     */
    static void sndLauncher(void* arg)
    {
        reinterpret_cast<MavChannel*>(arg)->runSender();
    }

    // Sender and receiver
    Transceiver* device;
    MavHandler handleMavlinkMessage; // function to be executed when a message is rcv
    uint16_t sleep_after_send;
    miosix::Queue<mavlink_message_t, MAV_OUT_QUEUE_LEN> messageQueue;

    // Status
    MavStatus status;
    miosix::FastMutex mtx_status;

    // Threads
    bool stop_flag = false;
    bool sndStarted = false;
    bool rcvStarted = false;
    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;
};