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

#include <kernel/scheduler/scheduler.h>
#include <miosix.h>
#include "Canbus.h"

namespace Boardcore
{
namespace Canbus
{
Canbus* can_drivers[2];
}
}  // namespace Boardcore

void __attribute__((naked)) CAN1_RX0_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z20CAN_RXIRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN1_RX1_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z20CAN_RXIRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX0_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z20CAN_RXIRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX1_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z20CAN_RXIRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN1_TX_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("bl _Z20CAN_TXIRQHandlerImpli");
    restoreContext();
}

void __attribute__((naked)) CAN2_TX_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("bl _Z20CAN_TXIRQHandlerImpli");
    restoreContext();
}

void __attribute__((used)) CAN_RXIRQHandlerImpl(int can_dev, int fifo)
{
    using namespace Canbus;
    (void)can_dev;

    if (can_drivers[can_dev])
    {
        can_drivers[can_dev]->handleRXInterrupt(fifo);
    }
}

void __attribute__((used)) CAN_TXIRQHandlerImpl(int can_dev)
{
    (void)can_dev;
    bool hppw = false;

    using namespace Canbus;

    Canbus::Canbus* bus = can_drivers[can_dev];

    if (bus)
    {
        CAN_TypeDef* can = bus->getCAN();

        CanTXResult res;
        res.seq      = bus->getLastTXSeq();
        res.tme      = can->TSR & CAN_TSR_TME >> 26;
        res.err_code = (can->ESR | CAN_ESR_LEC) >> 4;

        if ((can->TSR & CAN_TSR_RQCP0) > 0)
        {
            res.mailbox = 0;
            res.tx_status =
                can->TSR & (CAN_TSR_TXOK0 | CAN_TSR_ALST0 | CAN_TSR_TERR0) >> 1;

            can->TSR |= CAN_TSR_RQCP0;
        }
        if ((can->TSR & CAN_TSR_RQCP1) > 0)
        {
            res.mailbox = 1;
            res.tx_status =
                can->TSR & (CAN_TSR_TXOK1 | CAN_TSR_ALST1 | CAN_TSR_TERR1) >> 9;

            can->TSR |= CAN_TSR_RQCP1;
        }
        if ((can->TSR & CAN_TSR_RQCP2) > 0)
        {
            res.mailbox = 2;
            res.tx_status =
                can->TSR & (CAN_TSR_TXOK2 | 2 | CAN_TSR_TERR2) >> 17;

            can->TSR |= CAN_TSR_RQCP2;
        }

        bus->getTXResultBuffer().IRQput(res, hppw);
        bus->wakeTXThread();
    }

    if (hppw)
        miosix::Scheduler::IRQfindNextThread();
}

