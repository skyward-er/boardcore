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

/* WARNING: due to this guard, moving implementation into a .cpp file won't
 * compile */
#ifndef MAVLINK_H
#error Wrong include order: you should include a MAVLINK_H before using this
#endif

#include <Common.h>
#include <drivers/Transceiver.h>
#include "MavStatus.h"
#include "utils/SyncedCircularBuffer.h"

static constexpr int MAV_OUT_QUEUE_LEN             = 10;
static constexpr int MAV_OUT_BUFFER_SIZE           = 512;
static constexpr int MAV_IN_BUFFER_SIZE            = 512;
static constexpr int MAV_OUT_SEND_BUFFER_THRESHOLD = 230;
static constexpr long long MAV_OUT_BUFFER_MAX_AGE  = 1500;

/**
 * Class to parse mavlink messages through a Transceiver. Lets you
 * choose the function to execute when a message is received.
 */
class MavChannel
{

    /* Alias of the function to be executed on message rcv */
    using MavHandler =
        std::function<void(MavChannel* channel, const mavlink_message_t& msg)>;

public:
    /**
     * @param device     device used to send and receive
     * @param onRcv      function to be executed on message rcv
     * @param sleepTime  min sleep after send
     */
    MavChannel(Transceiver* device, MavHandler onRcv, uint16_t sleepTime = 0)
        : device(device), handleMavlinkMessage(onRcv),
          sleep_after_send(sleepTime)
    {
        memset(&status, 0, sizeof(MavStatus));
        memset(out_buffer, 0, MAV_OUT_BUFFER_SIZE);
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
            sndThread = miosix::Thread::create(
                sndLauncher, miosix::STACK_DEFAULT_FOR_PTHREAD,
                miosix::MAIN_PRIORITY, reinterpret_cast<void*>(this),
                miosix::Thread::JOINABLE);

            if (sndThread != nullptr)
                sndStarted = true;
        }

        // Start receiver
        if (!rcvStarted)
        {
            rcvThread = miosix::Thread::create(
                rcvLauncher, miosix::STACK_DEFAULT_FOR_PTHREAD,
                miosix::MAIN_PRIORITY, reinterpret_cast<void*>(this));

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
    }

    /**
     * Reads one char at a time from the device and tries to parse a mavlink
     * message. If the message is successfully parsed, executes the function
     * that was passed in the constructor.
     */
    void runReceiver()
    {
        mavlink_message_t msg;
        uint8_t rcv_buffer[MAV_IN_BUFFER_SIZE];

        while (!stop_flag)
        {
            ssize_t rcv_size =
                device->receive(rcv_buffer,
                                MAV_IN_BUFFER_SIZE);  // Blocking function
            uint8_t parse_result = 0;

            miosix::Lock<miosix::FastMutex> l(mtx_status);
            for (ssize_t i = 0; i < rcv_size; i++)
            {
                // Parse received bytes
                parse_result = mavlink_parse_char(MAVLINK_COMM_0, rcv_buffer[i],
                                                  &msg, &(status.mav_stats));
                if (parse_result == 1)
                {
                    miosix::Unlock<miosix::FastMutex> unlock(l);

                    TRACE(
                        "[MAV] Received message with ID %d, sequence: %d from "
                        "component %d of system %d\n",
                        msg.msgid, msg.seq, msg.compid, msg.sysid);

                    /* Handle the command */
                    handleMavlinkMessage(this, msg);
                }
            }
        }
    }

    /**
     * Periodically flushes the message queue and sends all the enqueued
     * messages. After every send, a sleep is done to guarantee some silence on
     * the channel.
     */
    void runSender()
    {
        TRACE("[MAV] Sender is running\n");

        while (!stop_flag)
        {
            while (!message_queue.isEmpty())
            {
                mavlink_message_t msgTemp = message_queue.pop();
                int msgLen                = mavlink_msg_to_send_buffer(
                    out_buffer + out_buffer_size, &msgTemp);
                out_buffer_size += msgLen;

                if (out_buffer_size >= MAV_OUT_SEND_BUFFER_THRESHOLD)
                {
                    break;
                }
            }
            if (out_buffer_size > 0)
            {
                if (out_buffer_size >= MAV_OUT_SEND_BUFFER_THRESHOLD ||
                    out_buffer_age >= MAV_OUT_BUFFER_MAX_AGE)
                {
                    sendBuffer();
                }

                out_buffer_age += sleep_after_send;
            }

            miosix::Thread::sleep(sleep_after_send);         
        }
    }

    void sendBuffer()
    {
        bool sent       = device->send(out_buffer, out_buffer_size);
        out_buffer_age  = 0;
        out_buffer_size = 0;

        TRACE("[MAV] Sending %d bytes\n", out_buffer_size);

        if (!sent)
            TRACE("[MAV] Error: could not send message\n");

        /* Update status */
        {
            miosix::Lock<miosix::FastMutex> l(mtx_status);

            status.n_send_queue = message_queue.count();
            status.mav_stats.current_tx_seq++;

            if (!sent)
                status.n_send_errors++;
        }
    }

    /**
     * Non-blocking send function. Message is put in the queue if not full.
     * @param msg    message to send (mavlink struct)
     * @return true if the message could be enqueued (queue not full)
     */
    bool enqueueMsg(const mavlink_message_t& msg)
    {
        {
            miosix::Lock<miosix::FastMutex> l(mtx_status);

            if (message_queue.isFull())
            {
                status.n_dropped_packets++;
            }
            else
            {
                status.n_send_queue = message_queue.count();
                if (status.n_send_queue > status.max_send_queue)
                    status.max_send_queue = status.n_send_queue;
            }
        }

        message_queue.put(msg);
        TRACE("[MAV] Enqueueing %d bytes\n", msg.len);
        return true;
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
    MavHandler
        handleMavlinkMessage;  // function to be executed when a message is rcv
    uint16_t sleep_after_send;

    SyncedCircularBuffer<mavlink_message_t, MAV_OUT_QUEUE_LEN> message_queue;

    uint8_t out_buffer[MAV_OUT_BUFFER_SIZE];
    unsigned int out_buffer_size = 0;
    long long out_buffer_age     = 0;

    // Status
    MavStatus status;
    miosix::FastMutex mtx_status;

    // Threads
    bool stop_flag  = false;
    bool sndStarted = false;
    bool rcvStarted = false;

    miosix::Thread* sndThread = nullptr;
    miosix::Thread* rcvThread = nullptr;
};