/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include <drivers/Transceiver.h>
#include "MavReceiver.h"
#include "MavSender.h"
#include "../MavStatus.h"

/**
 * This class is meant to simply manage a set of mavlink senders and receivers.
 * The idea is that there might be more than one tranceiver to manage, for redundancy
 * reasons or for having different channels for TCs and TMs.
 */
class MavManager
{

public:

    MavManager()
    {
        memset(&status, 0, sizeof(status));
    }
    
    ~MavManager()
    {
        /* Destroy senders */
        while (senders.size() > 0)
        {
            delete senders[senders.size() - 1];
            senders.pop_back();
        }

        /* Destroy receivers */
        while (receivers.size() > 0)
        {
            delete receivers[receivers.size() - 1];
            receivers.pop_back();
        }
    }

    uint16_t addSender(Transceiver* device, uint16_t sleepTime)
    {
        MavSender* sender = new MavSender(device, &status, sleepTime);
        senders.push_back(sender);
        sender->start();

        return senders.size();
    }


    uint16_t addReceiver(Transceiver* device, MavSender* sender, MavHandler onRcv)
    {
        MavReceiver* receiver = new MavReceiver(device, sender, onRcv, &status);
        receivers.push_back(receiver);
        receiver->start();

        return receivers.size();
    }


    MavSender* getSender(uint16_t id) 
    {
        return senders[id];
    }


    MavReceiver* getReceiver(uint16_t id)
    {
        return receivers[id];
    }

    /** WARNING: not synchronized **/
    MavStatus getStatus()
    {
        loggable_status.timestamp = miosix::getTick();
        loggable_status.mav_stats = status;
        return loggable_status;
    }

private:

    std::vector<MavSender *> senders;
    std::vector<MavReceiver *> receivers;

    mavlink_status_t status;
    MavStatus loggable_status;
};
