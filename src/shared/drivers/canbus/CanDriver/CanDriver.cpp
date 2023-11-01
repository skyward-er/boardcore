/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include "CanDriver.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/ClockUtils.h>

#include <algorithm>
#include <cmath>

#include "CanInterrupt.h"
#include "diagnostic/PrintLogger.h"

namespace Boardcore
{

namespace Canbus
{

PrintLogger l = Logging::getLogger("canbus");

CanbusDriver::CanbusDriver(CAN_TypeDef* can, CanbusConfig config,
                           AutoBitTiming bitTiming)
    : CanbusDriver(can, config, calcBitTiming(bitTiming))
{
}

CanbusDriver::CanbusDriver(CAN_TypeDef* can, CanbusConfig config,
                           BitTiming bitTiming)
    : can(can)
{
    if (can == CAN2)
    {
        // CAN2 also need the CAN1 clock
        ClockUtils::enablePeripheralClock(CAN1);
    }
    // Enable the peripheral clock
    ClockUtils::enablePeripheralClock(can);

    // Enter init mode
    can->MCR &= ~CAN_MCR_SLEEP;
    can->MCR |= CAN_MCR_INRQ;

    while ((can->MSR & CAN_MSR_INAK) == 0)
        ;

    // Automatic wakeup when a new packet is available
    if (config.awum)
        can->MCR |= CAN_MCR_AWUM;

    // Automatically recover from Bus-Off mode
    if (config.abom)
        can->MCR |= CAN_MCR_ABOM;

    // Disable automatic retransmission
    if (config.nart)
        can->MCR |= CAN_MCR_NART;

    // Bit timing configuration
    can->BTR &= ~CAN_BTR_BRP;
    can->BTR &= ~CAN_BTR_TS1;
    can->BTR &= ~CAN_BTR_TS2;
    can->BTR &= ~CAN_BTR_SJW;

    can->BTR |= bitTiming.BRP & 0x3FF;
    can->BTR |= ((bitTiming.BS1 - 1) & 0xF) << 16;
    can->BTR |= ((bitTiming.BS2 - 1) & 0x7) << 20;
    can->BTR |= ((bitTiming.SJW - 1) & 0x3) << 24;

    if (config.loopback)
        can->BTR |= CAN_BTR_LBKM;

    // Enter filter initialization mode
    can->FMR |= CAN_FMR_FINIT;

    if (can == CAN1)
        canDrivers[0] = this;
    else
        canDrivers[1] = this;

    // Enable interrupts
    can->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE;

    // Enable the corresponding interrupts
    if (can == CAN1)
    {
        NVIC_EnableIRQ(CAN1_RX0_IRQn);
        NVIC_SetPriority(CAN1_RX0_IRQn, 14);

        NVIC_EnableIRQ(CAN1_RX1_IRQn);
        NVIC_SetPriority(CAN1_RX1_IRQn, 14);

        NVIC_EnableIRQ(CAN1_TX_IRQn);
        NVIC_SetPriority(CAN1_TX_IRQn, 14);
    }
    else if (can == CAN2)
    {
        NVIC_EnableIRQ(CAN2_RX0_IRQn);
        NVIC_SetPriority(CAN2_RX0_IRQn, 14);

        NVIC_EnableIRQ(CAN2_RX1_IRQn);
        NVIC_SetPriority(CAN2_RX1_IRQn, 14);

        NVIC_EnableIRQ(CAN2_TX_IRQn);
        NVIC_SetPriority(CAN2_TX_IRQn, 14);
    }
    else
    {
        PrintLogger ls = l.getChild("constructor");
        LOG_ERR(ls, "Not supported peripheral");
    }
}

CanbusDriver::~CanbusDriver()
{
    ClockUtils::disablePeripheralClock(can);
    if (can == CAN2)
    {
        ClockUtils::disablePeripheralClock(CAN1);
    }
}

CanbusDriver::BitTiming CanbusDriver::calcBitTiming(AutoBitTiming autoBt)
{
    PrintLogger ls = l.getChild("bittiming");

    BitTiming cfgOpt;
    float costOpt = 1000;
    uint8_t NOpt  = 5;

    BitTiming cfgIter;
    cfgIter.SJW     = 0;
    uint32_t apbclk = ClockUtils::getAPBPeripheralsClock(ClockUtils::APB::APB1);

    // Iterate over the possible number of quanta in a bit to find the best
    // settings
    for (uint8_t N = 3; N <= 25; N++)
    {
        // Calc optimal baud rate prescaler
        cfgIter.BRP = std::max(
            std::min((int)roundf(apbclk * 1.0f / (autoBt.baudRate * N) - 1),
                     1 << 10),
            1);

        // Given N, calculate BS1 and BS2 that result in a sample time as
        // close as possible to the target one
        cfgIter.BS1 =
            std::min(std::max((int)roundf(autoBt.samplePoint * N - 1), 1),
                     std::min(N - 2, 16));

        cfgIter.BS2 = N - cfgIter.BS1 - 1;

        float brErrPercent =
            fabs(apbclk * 1.0f / (N * (cfgIter.BRP + 1)) - autoBt.baudRate) /
            autoBt.baudRate;
        float sp = (1 + cfgIter.BS1) * 1.0f / (1 + cfgIter.BS1 + cfgIter.BS2);

        float spErrPercent = fabs(sp - autoBt.samplePoint) / autoBt.samplePoint;

        // Calculate the cost function over N
        float cost = BR_ERR_WEIGHT * brErrPercent +
                     SP_ERR_WEIGHT * spErrPercent +
                     N_ERR_WEIGHT * fabs(N - 10) / 25;

        // Find config that minimizes the cost function
        if (cost < costOpt)
        {
            cfgOpt  = cfgIter;
            costOpt = cost;
            NOpt    = N;
        }
    }

    cfgOpt.SJW = fminf(ceilf(NOpt * 1.0f / 5), 4);

    LOG_DEBUG(ls,
              "Optimal Bit Timing Registers: BRP={}, BS1={}, BS2={}, SJW={}",
              cfgOpt.BRP, cfgOpt.BS1, cfgOpt.BS2, cfgOpt.SJW);

    float brTrue = apbclk * 1.0f / ((cfgOpt.BRP + 1) * NOpt);
    float spTrue = (1 + cfgOpt.BS1) * 1.0f / (1 + cfgOpt.BS1 + cfgOpt.BS2);

    LOG_DEBUG(ls,
              "Optimal Bit Timing: BR_true={:.2f}, spTrue:{:.2f}%, "
              "BR_error={:.2f}%, SP_error={:.2f}%",
              brTrue / 1000, spTrue * 100,
              fabsf(brTrue - autoBt.baudRate) / autoBt.baudRate * 100,
              fabs(spTrue - autoBt.samplePoint) / autoBt.samplePoint * 100);

    return cfgOpt;
}

void CanbusDriver::init()
{
    if (isInit)
        return;

    PrintLogger ls = l.getChild("init");

    can->FMR &= ~CAN_FMR_FINIT;  // Exit filter init mode
    can->MCR &= ~CAN_MCR_INRQ;   // Enable canbus

    // Wait until the can peripheral synchronizes with the bus

    LOG_DEBUG(ls, "Waiting for canbus synchronization...");
    while ((can->MSR & CAN_MSR_INAK) > 0)
        Thread::sleep(1);

    LOG_INFO(ls, "Canbus synchronized! Init done!");

    isInit = true;
}

bool CanbusDriver::addFilter(FilterBank filter)
{
    PrintLogger ls = l.getChild("addfilter");
    uint8_t index  = filterIndex;

    // CAN2 filters start from the 15th position
    if (can == CAN2)
    {
        index = index + 14;
    }

    if (isInit)
    {
        LOG_ERR(ls, "Cannot add filter: canbus already initialized");
        return false;
    }

    if (index >= NUM_FILTER_BANKS)
    {
        LOG_ERR(ls, "Cannot add filter: no more filter banks available");
        return false;
    }

    // NOTE: the filters are set in CAN1 peripheral because the filter registers
    // between the peripherals are in common.
    CAN1->sFilterRegister[index].FR1 = filter.FR1;
    CAN1->sFilterRegister[index].FR2 = filter.FR2;

    CAN1->FM1R |= (filter.mode == FilterMode::MASK ? 0 : 1) << index;
    CAN1->FS1R |= (filter.scale == FilterScale::DUAL16 ? 0 : 1) << index;
    CAN1->FFA1R |= (filter.fifo & 0x1) << index;

    // Enable the filter
    CAN1->FA1R |= 1 << index;

    ++filterIndex;

    return true;
}

uint32_t CanbusDriver::send(CanPacket packet)
{
    PrintLogger ls = l.getChild("send");

    if (!isInit)
    {
        LOG_ERR(ls, "Canbus is not initialized!");
        return 0;
    }

    bool didWait = false;

    {
        miosix::FastInterruptDisableLock d;

        // Wait until there is an empty mailbox available to use
        while ((can->TSR & CAN_TSR_TME) == 0)
        {
            didWait = true;
            waiting = Thread::IRQgetCurrentThread();
            Thread::IRQwait();
            {
                miosix::FastInterruptEnableLock e(d);
                Thread::yield();
            }
        }
    }

    if (didWait)
    {
        // Warn that the function blocked. We are probably transmitting too fast
        LOG_WARN_ASYNC(ls, "Had to wait for an empty mailbox!");
    }

    // Index of first empty mailbox
    uint8_t mbxCode = (can->TSR & CAN_TSR_CODE) >> 24;

    if (mbxCode > 2)
    {
        LOG_ERR(ls, "Error! Invalid TSR_CODE!");
        return 0;
    }

    uint32_t seq          = txSeq++;
    txMailboxSeq[mbxCode] = seq;

    CAN_TxMailBox_TypeDef* mailbox = &can->sTxMailBox[mbxCode];

    can->sTxMailBox[mbxCode].TIR &= CAN_TI0R_TXRQ;
    if (packet.ext)
    {
        can->sTxMailBox[mbxCode].TIR |= ((packet.id & 0x1FFFFFFF) << 3);
        can->sTxMailBox[mbxCode].TIR |= 1 << 2;
    }
    else
    {
        can->sTxMailBox[mbxCode].TIR |= ((packet.id & 0x7FF) << 21);
    }

    can->sTxMailBox[mbxCode].TIR |= (packet.rtr ? 1 : 0) << 1;

    mailbox->TDTR = (packet.length & 0xF);

    // Reset data registers
    mailbox->TDLR = 0;
    mailbox->TDHR = 0;

    // Fill data registers
    for (uint8_t i = 0; i < packet.length; ++i)
    {
        if (i < 4)
            mailbox->TDLR |= packet.data[i] << i * 8;
        else
            mailbox->TDHR |= packet.data[i] << (i - 4) * 8;
    }

    // Finally send the packet
    can->sTxMailBox[mbxCode].TIR |= CAN_TI0R_TXRQ;

    return seq;
}

void CanbusDriver::handleRXInterrupt(int fifo)
{
    CanRXStatus status;
    status.fifo = fifo;
    volatile uint32_t* RFR;
    CAN_FIFOMailBox_TypeDef* mailbox;

    mailbox = &can->sFIFOMailBox[fifo];
    if (fifo == 0)
        RFR = &can->RF0R;
    else
        RFR = &can->RF1R;

    status.fifoOverrun = (*RFR & CAN_RF0R_FOVR0) > 0;
    status.fifoFull    = (*RFR & CAN_RF0R_FULL0) > 0;

    CanPacket p;
    bool hppw = false;

    // Note: Bit position definitions are the same for both FIFOs
    // eg: CAN_RF0R_FMP0 == CAN_RF1R_FMP1

    if ((*RFR & CAN_RF0R_FMP0) > 0)
    {
        p.timestamp = miosix::getTime() / 1e6;

        status.rxStatus     = *RFR & (CAN_RF0R_FULL0 | CAN_RF0R_FOVR0) >> 3;
        status.errCode      = (can->ESR | CAN_ESR_LEC) >> 4;
        status.rxErrCounter = (can->ESR | CAN_ESR_REC) >> 24;

        p.ext = (mailbox->RIR & CAN_RI0R_IDE) > 0;
        p.rtr = (mailbox->RIR & CAN_RI0R_RTR) > 0;

        if (p.ext)
            p.id = (mailbox->RIR >> 3) & 0x1FFFFFFF;
        else
            p.id = (mailbox->RIR >> 21) & 0x7FF;

        p.length = mailbox->RDTR & CAN_RDT0R_DLC;

        for (uint8_t i = 0; i < p.length; i++)
        {
            if (i < 4)  // Low register
                p.data[i] = (mailbox->RDLR >> (i * 8)) & 0xFF;
            else  // High register
                p.data[i] = (mailbox->RDHR >> ((i - 4) * 8)) & 0xFF;
        }

        *RFR |= CAN_RF0R_RFOM0;

        // Put the message into the queue
        bufRxPackets.IRQput(CanRXPacket{p, status}, hppw);
    }

    if (hppw)
        miosix::Scheduler::IRQfindNextThread();
}

void CanbusDriver::wakeTXThread()
{
    if (waiting)
    {
        waiting->IRQwakeup();
        waiting = 0;
    }
}

}  // namespace Canbus

}  // namespace Boardcore
