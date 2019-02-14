/* Copyright (c) 2018 Skyward Experimental Rocketry
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
 *
 */

/**
 * This file provides a simplified way to initialize the NoseconeBoard and 
 * include all needed headers. This is mainly for debug pourposes.
 */
#pragma once

#include <Common.h>

#include <boards/Nosecone/FSM/NoseconeManager.h>
#include <events/EventBroker.h>

#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

#include <boards/Nosecone/LogProxy/LogProxy.h>
#include <boards/CanInterfaces.h>
#include <boards/Nosecone/Canbus/CanImpl.h>
#include <PinObserver.h>


using namespace NoseconeBoard;
using namespace miosix;
using namespace CanInterfaces;

class NoseconeBoardImpl
{

public:
    PinObserver pinObs;
    MotorDriver motor;

    CanManager canMgr;
    NoseconeManager noseconeMgr;
    LoggerProxy* logger;

    NoseconeBoardImpl() : motor(&pinObs), canMgr(CAN1), noseconeMgr(motor)
    {
        /* Global environment */
        sEventBroker->start();
        logger = Singleton<LoggerProxy>::getInstance();
        pinObs.start();

        /* Canbus management */
        CanImpl::initCanbus(canMgr);

        /* FSM */
        noseconeMgr.start();
    }

    inline void postEvent(Event ev) 
    {
        sEventBroker->post(ev, TOPIC_NOSECONE);
    }

    inline NoseconeBoardStatus getStatus()
    {
        return logger->getNoseconeStatus();
    }
};