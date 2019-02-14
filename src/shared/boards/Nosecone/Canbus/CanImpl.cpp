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

#include "CanImpl.h"

#include <events/EventBroker.h>

#include <boards/CanInterfaces.h>
#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>
#include <boards/Nosecone/LogProxy/LogProxy.h>

using namespace CanInterfaces;

namespace NoseconeBoard
{
namespace CanImpl
{

/**
 * Canbus receiving function.
 */
void canRcv(CanMsg message, CanManager* c) 
{

    TRACE("[CAN] Received message with id %lu\n", message.StdId);

    if (message.StdId == CAN_TOPIC_HOMEONE)
    {
        switch(message.Data[0]) 
        {
            case CAN_MSG_NSC_OPEN:
                sEventBroker->post(Event{EV_OPEN}, TOPIC_NOSECONE);
                break;

            case CAN_MSG_NSC_CLOSE:
                sEventBroker->post(Event{EV_CLOSE}, TOPIC_NOSECONE);
                break;

            case CAN_MSG_NSC_STOP:
                sEventBroker->post(Event{EV_STOP}, TOPIC_NOSECONE);
                break;

            case CAN_MSG_REQ_NSC_STATUS: 
            {
                /* Retrieve and send status */
                NoseconeBoardStatus st = 
                        LoggerProxy::getInstance()->getNoseconeStatus();
                canSendNoseconeStatus(c->getBus(0), st);

                /* Reset timer */
                canStatus.homeone_not_connected = 0;
                // TODO reset timer & log
                break;
            }

            default:
                TRACE("[CAN] Error: bad command\n");
        }
    }
    else 
    {
        TRACE("[CAN] Error: bad message id\n");
    }
}


void initCanbus(CanManager& c)
{
    using namespace std::placeholders;

    /* Subscribe to homeone commands */
    c.addHWFilter(CAN_TOPIC_HOMEONE, 0);

    /* Create init structure */
    canbus_init_t st = {
        CAN1, miosix::Mode::ALTERNATE, 9, {CAN1_RX0_IRQn, CAN1_RX1_IRQn}};
 
    /* Bind the receiving function to a bus (see std::bind reference) */
    auto dispatch = std::bind(canRcv, _1, &c);

    /* Init bus */
    c.addBus<GPIOA_BASE, 11, 12>(st, dispatch);
    TRACE("[CAN] Initialised CAN1 on PA11-12 \n");
}

} /* namespace CanImpl */
} /* namespace NoseconeBoard */
