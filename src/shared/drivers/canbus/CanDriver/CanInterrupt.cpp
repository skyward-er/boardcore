/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Matteo Piazzolla, Alain Carlucci
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

#include "CanInterrupt.h"

#include <miosix.h>            // for CAN_TypeDef, etc.

#include "CanDriver.h"

using namespace miosix;

namespace Boardcore
{
namespace Canbus
{

// Global array of driver instances (set by the driver constructor)
CanbusDriver* canDrivers[2];

// ---------- Static interrupt handlers ----------
// Each handler calls the appropriate driver method.

static void CAN1_RX0_IRQHandler_wrapper()
{
    if (canDrivers[0])
        canDrivers[0]->handleRXInterrupt(0);
}

static void CAN1_RX1_IRQHandler_wrapper()
{
    if (canDrivers[0])
        canDrivers[0]->handleRXInterrupt(1);
}

static void CAN2_RX0_IRQHandler_wrapper()
{
    if (canDrivers[1])
        canDrivers[1]->handleRXInterrupt(0);
}

static void CAN2_RX1_IRQHandler_wrapper()
{
    if (canDrivers[1])
        canDrivers[1]->handleRXInterrupt(1);
}

static void CAN1_TX_IRQHandler_wrapper()
{
    if (canDrivers[0])
    {
        CanbusDriver* bus = canDrivers[0];
        CAN_TypeDef* can  = bus->getCAN();

        CanTXResult res;
        res.tme     = can->TSR & CAN_TSR_TME >> 26;
        res.errCode = (can->ESR | CAN_ESR_LEC) >> 4;

        if ((can->TSR & CAN_TSR_RQCP0) > 0)
        {
            res.mailbox = 0;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK0 | CAN_TSR_ALST0 | CAN_TSR_TERR0) >> 1;
            can->TSR |= CAN_TSR_RQCP0;
        }
        if ((can->TSR & CAN_TSR_RQCP1) > 0)
        {
            res.mailbox = 1;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK1 | CAN_TSR_ALST1 | CAN_TSR_TERR1) >> 9;
            can->TSR |= CAN_TSR_RQCP1;
        }
        if ((can->TSR & CAN_TSR_RQCP2) > 0)
        {
            res.mailbox = 2;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK2 | 2 | CAN_TSR_TERR2) >> 17;
            can->TSR |= CAN_TSR_RQCP2;
        }

        res.seq = bus->getTXMailboxSequence(res.mailbox);
        bus->getTXResultBuffer().IRQput(res);
        bus->wakeTXThread();
    }
}

static void CAN2_TX_IRQHandler_wrapper()
{
    if (canDrivers[1])
    {
        CanbusDriver* bus = canDrivers[1];
        CAN_TypeDef* can  = bus->getCAN();

        CanTXResult res;
        res.tme     = can->TSR & CAN_TSR_TME >> 26;
        res.errCode = (can->ESR | CAN_ESR_LEC) >> 4;

        if ((can->TSR & CAN_TSR_RQCP0) > 0)
        {
            res.mailbox = 0;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK0 | CAN_TSR_ALST0 | CAN_TSR_TERR0) >> 1;
            can->TSR |= CAN_TSR_RQCP0;
        }
        if ((can->TSR & CAN_TSR_RQCP1) > 0)
        {
            res.mailbox = 1;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK1 | CAN_TSR_ALST1 | CAN_TSR_TERR1) >> 9;
            can->TSR |= CAN_TSR_RQCP1;
        }
        if ((can->TSR & CAN_TSR_RQCP2) > 0)
        {
            res.mailbox = 2;
            res.txStatus =
                can->TSR & (CAN_TSR_TXOK2 | 2 | CAN_TSR_TERR2) >> 17;
            can->TSR |= CAN_TSR_RQCP2;
        }

        res.seq = bus->getTXMailboxSequence(res.mailbox);
        bus->getTXResultBuffer().IRQput(res);
        bus->wakeTXThread();
    }
}

// ---------- Public registration functions ----------

void registerCanInterrupts(CAN_TypeDef* can)
{
    GlobalIrqLock lock;

    if (can == CAN1)
    {
        IRQregisterIrq(lock, CAN1_RX0_IRQn, &CAN1_RX0_IRQHandler_wrapper);
        IRQregisterIrq(lock, CAN1_RX1_IRQn, &CAN1_RX1_IRQHandler_wrapper);
        IRQregisterIrq(lock, CAN1_TX_IRQn,  &CAN1_TX_IRQHandler_wrapper);
    }
    else if (can == CAN2)
    {
        IRQregisterIrq(lock, CAN2_RX0_IRQn, &CAN2_RX0_IRQHandler_wrapper);
        IRQregisterIrq(lock, CAN2_RX1_IRQn, &CAN2_RX1_IRQHandler_wrapper);
        IRQregisterIrq(lock, CAN2_TX_IRQn,  &CAN2_TX_IRQHandler_wrapper);
    }
}

void unregisterCanInterrupts(CAN_TypeDef* can)
{
    GlobalIrqLock lock;

    if (can == CAN1)
    {
        IRQunregisterIrq(lock, CAN1_RX0_IRQn, &CAN1_RX0_IRQHandler_wrapper);
        IRQunregisterIrq(lock, CAN1_RX1_IRQn, &CAN1_RX1_IRQHandler_wrapper);
        IRQunregisterIrq(lock, CAN1_TX_IRQn,  &CAN1_TX_IRQHandler_wrapper);
    }
    else if (can == CAN2)
    {
        IRQunregisterIrq(lock, CAN2_RX0_IRQn, &CAN2_RX0_IRQHandler_wrapper);
        IRQunregisterIrq(lock, CAN2_RX1_IRQn, &CAN2_RX1_IRQHandler_wrapper);
        IRQunregisterIrq(lock, CAN2_TX_IRQn,  &CAN2_TX_IRQHandler_wrapper);
    }
}

}  // namespace Canbus
}  // namespace Boardcore