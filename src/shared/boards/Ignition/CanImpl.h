/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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
#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanUtils.h>

#include "boards/CanInterfaces.h"
#include "IgnManager.h"

namespace IgnBoard
{

/**
 * Canbus receiving function.
 */
void canRcv(CanMsg message) 
{
    TRACE("[CAN] Received message with id %d\n", message.DLC);

    switch (message.StdId)
    {
        case CanInterfaces::CAN_TOPIC_HOMEONE:
            if(message.Data[0] == CanInterfaces::CAN_MSG_ABORT) {
                IgnitionManager::getInstance()->abort();
            } 
            else if(message.Data[0] == CanInterfaces::CAN_MSG_REQ_IGN_STATUS) {
                IgnitionManager::getInstance()->getStatus();
            } 
            else {
                TRACE("[CAN] Message not recognized\n");
            }
        break;

        case CanInterfaces::CAN_TOPIC_LAUNCH:
            uint64_t code = 0;
            memcpy(&code, message.Data, 8);

            IgnitionManager::getInstance()->launch(code);
        break;

        default:
            TRACE("[CAN] Message not recognized\n");
            break;
    }
}

/**
 * Initialise CAN1 on PA11, PA12, set filters and set receiver function.
 */
void initCanbus(CanManager& c)
{
    c.addHWFilter(CanInterfaces::CAN_TOPIC_HOMEONE, 0);
    c.addHWFilter(CanInterfaces::CAN_TOPIC_LAUNCH, 0);

    canbus_init_t st = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
    c.addBus<GPIOA_BASE, 11, 12>(st, &canRcv);

    // CanBus *bus = c.getBus(0);

    TRACE("[CAN] Initialised CAN1 on PA11-12 \n");
}

} /* namespace HomeoneBoard */
