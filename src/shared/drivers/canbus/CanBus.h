/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <ActiveObject.h>
#include <Common.h>
#include <diagnostic/PrintLogger.h>

#include "CanUtils.h"

/* Alias for the function to be executed on receive */
using CanDispatcher = std::function<void(CanMsg, CanStatus)>;

class CanManager;

/*
 * @brief Driver of the physical CAN bus.
 */
class CanBus : public ActiveObject
{

public:
    /*
     * @param bus        the physical pins to be used
     * @param manager    who created this bus object
     * @param can_id     the id of the currently created canbus (CAN1, CAN2...)
     * @param dispatcher function to be executed on receive
     */
    CanBus(CAN_TypeDef* bus, CanManager* manager, const int can_id,
           CanDispatcher dispatcher);
    ~CanBus() {}

    /* Receiving function executed by the ActiveObject */
    void rcvFunction();

    /* Sender function
     * @param id       Id of the message (aka topic)
     * @para message   message as byte array
     * @param len      length of the message (max 8 bytes, truncated if grater)
     * @return true    if the message was sent correctly
     */
    bool send(uint16_t id, const uint8_t* message, uint8_t len);

    /* Getters */
    volatile CAN_TypeDef* getBus() { return CANx; }
    CanStatus getStatus();

    virtual void stop() override
    {
        should_stop = true;

        // Wake the thread posting an empty can msg
        CanMsg empty_msg;
        empty_msg.StdId = 0xFFFFFFFF;
        empty_msg.ExtId = 0xFFFFFFFF;
        rcvQueue.put(empty_msg);

        ActiveObject::stop();
    }

    /* Rule of five */
    CanBus(const CanBus&)  = delete;
    CanBus(const CanBus&&) = delete;
    CanBus& operator=(const CanBus&) = delete;

    /* Received messages queue, populated by interrupt */
    miosix::Queue<CanMsg, 6> rcvQueue;

protected:
    /* Inherited from ActiveObject: executes the receiving function */
    void run() override { rcvFunction(); }

private:
    void canSetup();

    volatile CAN_TypeDef* CANx;
    CanManager* manager;
    const int id;

    miosix::FastMutex sendMutex;
    miosix::FastMutex statusMutex;

    CanDispatcher dispatchMessage;
    CanStatus status;

    PrintLogger logger = Logging::getLogger("canbus");
};
