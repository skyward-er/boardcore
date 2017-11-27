/* CAN-Bus Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla, Alain Carlucci
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

#include "CanBus.h"

using namespace miosix;

CanBus *global_bus_ptr[2] = {NULL, NULL};
uint32_t global_bus_ctr   = 0;

void __attribute__((naked)) CAN1_RX0_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN1_RX1_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #0");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX0_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #0");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((naked)) CAN2_RX1_IRQHandler()
{
    saveContext();
    asm volatile("mov r0, #1");
    asm volatile("mov r1, #1");
    asm volatile("bl _Z18CAN_IRQHandlerImplii");
    restoreContext();
}

void __attribute__((used)) CAN_IRQHandlerImpl(int can_dev, int fifo)
{
    CanBus *hlbus;
    CanMsg RxMessage;
    volatile CAN_TypeDef *can;

    // per prima cosa "resetto" l'interrupt altrimenti entro in un ciclo
    //  CAN1->RF0R |= CAN_RF0R_RFOM0; non è lui xD
    can_dev &= 0x01;

    // TODO: check if this 'return' is allowed
    if (can_dev >= (int)global_bus_ctr)
        return;

    hlbus = global_bus_ptr[can_dev];
    can   = hlbus->getBus();

    RxMessage.IDE = (uint8_t)0x04 & can->sFIFOMailBox[fifo].RIR;
    if (RxMessage.IDE == CAN_ID_STD)
        RxMessage.StdId =
            (uint32_t)0x000007FF & (can->sFIFOMailBox[fifo].RIR >> 21);
    else
        RxMessage.ExtId =
            (uint32_t)0x1FFFFFFF & (can->sFIFOMailBox[fifo].RIR >> 3);

    RxMessage.RTR = (uint8_t)0x02 & can->sFIFOMailBox[fifo].RIR;
    RxMessage.DLC = (uint8_t)0x0F & can->sFIFOMailBox[fifo].RDTR;
    RxMessage.FMI = (uint8_t)0xFF & (can->sFIFOMailBox[fifo].RDTR >> 8);

    /* Get the data field */
    *((uint32_t *)RxMessage.Data)       = can->sFIFOMailBox[fifo].RDLR;
    *((uint32_t *)(RxMessage.Data + 4)) = can->sFIFOMailBox[fifo].RDHR;

    /* Release FIFO0 */
    can->RF0R |= CAN_RF0R_RFOM0;

    bool hppw = false;

    hlbus->messageQueue.IRQput(RxMessage, hppw);

    if (hppw)
        Scheduler::IRQfindNextThread();
}
