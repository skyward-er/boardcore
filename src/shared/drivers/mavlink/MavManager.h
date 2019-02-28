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

#include <Common.h>

#include <drivers/Transceiver.h>
#include "MavReceiver.h"
#include "MavSender.h"

/**
 * This class is meant to simply manage a set of mavlink senders and receivers.
 * The idea is that there might be more than one tranceiver to manage, for redundancy
 * reasons or for having different channels for TCs and TMs.
 */
class MavManager
{

public:

    MavManager(){}
    ~MavManager();

    uint16_t addSender(Transceiver* device, uint16_t sleepTime);
    uint16_t addReceiver(Transceiver* device, MavSender* sender, MavHandler onRcv);

    MavSender* getSender(uint16_t id);
    MavReceiver* getReceiver(uint16_t id);

private:

    std::vector<MavSender *> senders;
    std::vector<MavReceiver *> receivers;
};