/* Mavlink receiver
 *
 * Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#ifndef MAVLINK_H
    #error Wrong include order: you should include a MAVLINK_H before using this
#endif

#include <Common.h>
#include <ActiveObject.h>
#include <drivers/Transceiver.h>

#include <libs/mavlink_skyward_lib/mavlink_lib/protocol.h>

#include "MavSender.h"

/* Alias of the function to be executed whenever a message is correctly parsed */
using MavHandler = std::function<void(MavSender* sender, const mavlink_message_t& msg)>;

/**
 * Class to parse mavlink messages through a Transceiver. Lets you
 * choose the function to execute when a message is received.
 */
class MavReceiver : public ActiveObject
{
public:
    /**
     * @param device   the device used for receiving messages
     * @param onRcv    function to be exectued on message reception 
     */
    MavReceiver(Transceiver* device, MavSender* sender, MavHandler onRcv) : 
                                            ActiveObject(), 
                                            device(device), 
                                            sender(sender),
                                            handleMavlinkMessage(onRcv) {}
    
    ~MavReceiver(){};

    mavlink_status_t mavStatus; // updated every message

protected:
    /**
     * Inherited from ActiveObject: reads one char at a time from
     * the device and tries to parse a mavlink message.
     * If the message is successfully parsed, executes the function that was
     * passed in the constructor.
     */
    void run() override
    {
        mavlink_message_t msg;
        uint8_t byte;

        while(true)
        {
            device->receive(&byte, 1);  // Blocking function

            /* If a complete mavlink message was found */
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &mavStatus))
            {
                TRACE("[MAV] Received message with ID %d, sequence: %d from "
                    "component %d of system %d\n",
                    msg.msgid, msg.seq, msg.compid, msg.sysid);

                /* Handle the command */
                handleMavlinkMessage(sender, msg);
            }
        }
    }

private:
    Transceiver* device;
    MavSender* sender;               // needed to send back a response
    MavHandler handleMavlinkMessage; // function to be executed when a message is parsed
};