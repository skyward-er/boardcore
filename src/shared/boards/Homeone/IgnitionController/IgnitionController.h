/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "IgnitionStatus.h"
#include "Singleton.h"
#include "events/FSM.h"
#include <boards/Homeone/LogProxy/LogProxy.h>
#include "drivers/canbus/CanManager.h"
#include "drivers/canbus/CanUtils.h"
#include "boards/Homeone/EventClasses.h"

class CanEventSocket;

namespace HomeoneBoard
{

class IgnitionController : public FSM<IgnitionController>
{
public:
    explicit IgnitionController(CanBus* canbus);
    ~IgnitionController() {}

    IgnitionStatus getStatus() { return status; }
private:
    void stateIdle(const Event& ev);
    void stateAborted(const Event& ev);
    void stateEnd(const Event& ev);

    /**
     * @brief Updates the status of the ignition board if received on the canbus
     *
     * @param ev ev The event notifying a new message on the canbus
     * @return Wether the board status was updated or not
     */
    bool updateIgnBoardStatus(const Event& ev);

    IgnitionStatus status;

    LoggerProxy& logger = *(LoggerProxy::getInstance());

    uint16_t ev_ign_offline_handle = 0;
    uint16_t ev_get_status_handle  = 0;

    CanBus* canbus;
};

}  // namespace HomeoneBoard