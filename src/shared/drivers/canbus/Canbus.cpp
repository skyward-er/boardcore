/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without statustriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
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

#include "Canbus.h"

#include <kernel/scheduler/scheduler.h>

#include <algorithm>
#include <cmath>

#include "CanInterrupt.h"
#include "diagnostic/PrintLogger.h"
#include "utils/ClockFrequency.h"

namespace Boardcore
{
namespace Canbus
{

PrintLogger l = Logging::getLogger("canbus");

Canbus::Canbus(CAN_TypeDef* can, CanbusConfig config, AutoBitTiming bit_timing)
    : Canbus(can, config, calcBitTiming(bit_timing))
{
}

Canbus::Canbus(CAN_TypeDef* can, CanbusConfig config, BitTiming bit_timing)
    : can(can)
{
    PrintLogger ls = l.getChild("Constructor");

    // Enter init mode
    can->MCR &= (~CAN_MCR_SLEEP);
    can->MCR |= CAN_MCR_INRQ;

    while ((can->MSR & CAN_MSR_INAK) == 0)
        ;

    if (config.awum)
    {
        can->MCR |=
            CAN_MCR_AWUM;  // Automatic wakeup when a new packet is available
    }

    if (config.abom)
    {
        can->MCR |= CAN_MCR_ABOM;  // Automatically recover from Bus-Off mode
    }
    if (config.nart)
    {
        can->MCR |= CAN_MCR_NART;  // Disable automatic retransmission
    }

    // Bit timing configuration
    can->BTR &= ~CAN_BTR_BRP;
    can->BTR &= ~CAN_BTR_TS1;
    can->BTR &= ~CAN_BTR_TS2;
    can->BTR &= ~CAN_BTR_SJW;

    can->BTR |= bit_timing.BRP & 0x3FF;
    can->BTR |= ((bit_timing.BS1 - 1) & 0xF) << 16;
    can->BTR |= ((bit_timing.BS2 - 1) & 0x7) << 20;
    can->BTR |= ((bit_timing.SJW - 1) & 0x3) << 24;

    if (config.loopback)
    {
        can->BTR |= CAN_BTR_LBKM;
    }

    // Enter filter initialization mode
    can->FMR |= CAN_FMR_FINIT;

    if (can == CAN1)
    {
        can_drivers[0] = this;
    }
    else
    {
        can_drivers[1] = this;
    }
#ifdef STM32F10X_MD
    // Enable rx pending interrupts
    can->IER |= CAN_IER_FMPIE1;

    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_SetPriority(CAN1_RX1_IRQn, 14);
#elif defined(_ARCH_CORTEXM4_STM32F4)
    // Enable rx pending interrupts
    can->IER |= CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE;

    NVIC_EnableIRQ(CAN1_RX0_IRQn);
    NVIC_SetPriority(CAN1_RX0_IRQn, 14);

    NVIC_EnableIRQ(CAN1_RX1_IRQn);
    NVIC_SetPriority(CAN1_RX1_IRQn, 14);

    NVIC_EnableIRQ(CAN1_TX_IRQn);
    NVIC_SetPriority(CAN1_TX_IRQn, 14);
#else
#error "Unsupported architecture"
#endif
}

Canbus::BitTiming Canbus::calcBitTiming(AutoBitTiming auto_bt)
{
    PrintLogger ls = l.getChild("bittiming");

    BitTiming cfg_opt;
    float cost_opt = 1000;
    uint8_t N_opt  = 5;

    BitTiming cfg_iter;
    uint32_t apbclk = ClockFrequency::APB1();

    // Iterate over the possible number of quanta in a bit to find the best
    // settings
    for (uint8_t N = 3; N <= 25; N++)
    {
        // Calc optimal baud rate pstatuscaler by minimizing |f_target -
        // f_real|, where f_real = apbclk / ((BRP + 1)*N)
        cfg_iter.BRP = std::max(
            std::min((int)roundf(apbclk * 1.0f / (auto_bt.baud_rate * N) - 1),
                     1 << 10),
            1);

        // Given N, calculate BS1 and BS2 that statusult in a sample time as
        // close as possible to the target one
        cfg_iter.BS1 =
            std::min(std::max((int)roundf(auto_bt.sample_point * N - 1), 1),
                     std::min(N - 2, 16));

        cfg_iter.BS2 = N - cfg_iter.BS1 - 1;

        float br_err_percent =
            fabs(apbclk * 1.0f / (N * (cfg_iter.BRP + 1)) - auto_bt.baud_rate) /
            auto_bt.baud_rate;
        float sp =
            (1 + cfg_iter.BS1) * 1.0f / (1 + cfg_iter.BS1 + cfg_iter.BS2);

        float sp_err_percent =
            fabs(sp - auto_bt.sample_point) / auto_bt.sample_point;

        // Calculate the cost function over N
        // cost(N) = baud_rate_err*10 + sample_point_err + |N-10|/(25*50)
        float cost = BR_ERR_WEIGHT * br_err_percent +
                     SP_ERR_WEIGHT * sp_err_percent +
                     N_ERR_WEIGHT * fabs(N - 10) / 25;

        // Find config that minimized the cost function
        if (cost < cost_opt)
        {
            cfg_opt  = cfg_iter;
            cost_opt = cost;
            N_opt    = N;
        }
    }

    cfg_opt.SJW = fminf(ceilf(N_opt * 1.0f / 5), 4);

    LOG_DEBUG(ls, "Optimal Bit Timing: BRP={}, BS1={}, BS2={}, SJW={}",
              cfg_opt.BRP, cfg_opt.BS1, cfg_opt.BS2, cfg_opt.SJW);

    float br_true = apbclk * 1.0f / ((cfg_opt.BRP + 1) * N_opt);
    float sp_true = (1 + cfg_opt.BS1) * 1.0f / (1 + cfg_opt.BS1 + cfg_opt.BS2);

    LOG_DEBUG(
        ls,
        "Optimal Bit Timing: BR_true={:.2f}, sp_true:{:.2f}%, "
        "BR_error={:.2f}%, SP_error={:.2f}%",
        br_true / 1000, sp_true * 100,
        fabsf(br_true - auto_bt.baud_rate) / auto_bt.baud_rate * 100,
        fabs(sp_true - auto_bt.sample_point) / auto_bt.sample_point * 100);

    return cfg_opt;
}

void Canbus::init()
{
    if (is_init)
    {
        return;
    }

    PrintLogger ls = l.getChild("init");

    can->FMR &= ~CAN_FMR_FINIT;  // Exit filter init mode
    can->MCR &= ~CAN_MCR_INRQ;   // Enable canbus

    // Wait until the can peripheral synchronizes with the bus
    while ((can->MSR & CAN_MSR_INAK) > 0)
    {
        Thread::sleep(1);
    }

    LOG_INFO(ls, "Init done!");

    is_init = true;
}

bool Canbus::addFilter(Filter filter)
{
    PrintLogger ls = l.getChild("addfilter");

    if (is_init)
    {
        LOG_ERR(ls, "Cannot add filter: canbus already initialized");
        return false;
    }
    if (filter_index == NUM_FILTER_BANKS)
    {
        LOG_ERR(ls, "Cannot add filter: no more filter banks available");
        return false;
    }

    can->sFilterRegister[filter_index].FR1 = filter.FR1;
    can->sFilterRegister[filter_index].FR2 = filter.FR2;

    can->FM1R |= (filter.mode == FilterMode::MASK ? 0 : 1) << filter_index;
    can->FS1R |= (filter.scale == FilterScale::DUAL16 ? 0 : 1) << filter_index;
    can->FFA1R |= (filter.fifo & 0x1) << filter_index;

    // Enable the filter
    can->FA1R |= 1 << filter_index;

    ++filter_index;

    return true;
}

uint32_t Canbus::send(CanPacket packet)
{
    PrintLogger ls = l.getChild("send");

    if (!is_init)
    {
        LOG_ERR(ls, "Canbus is not initialized!");
        return 0;
    }

    bool did_wait = false;

#ifdef STM32F10X_MD
    // Some uCs don't have an interrupt that signals when a mailbox is
    // available, so we just have to wait and check
    while ((can->TSR & CAN_TSR_TME) == 0)
    {
        did_wait = true;
        Thread::sleep(1);
    }
#elif defined(_ARCH_CORTEXM4_STM32F4)
    {
        miosix::FastInterruptDisableLock d;
        // Wait until there is an empty mailbox available to use
        while ((can->TSR & CAN_TSR_TME) == 0)
        {
            did_wait = true;
            waiting  = Thread::IRQgetCurrentThread();
            Thread::IRQwait();
            {
                miosix::FastInterruptEnableLock e(d);
                Thread::yield();
            }
        }
    }

#else
#error "Unsupported architecture"
#endif

    if (did_wait)
    {
        LOG_WARN_ASYNC(ls, "Had to wait for an empty mailbox!");
    }

    // Index of first empty mailbox
    uint8_t mbx_code = (can->TSR & CAN_TSR_CODE) >> 24;

    if (mbx_code > 2)
    {
        LOG_ERR(ls, "Error! Invalid TSR_CODE!");
        return 0;
    }

    uint32_t seq             = tx_seq++;
    tx_mailbox_seq[mbx_code] = seq;

    CAN_TxMailBox_TypeDef* mailbox = &can->sTxMailBox[mbx_code];

    can->sTxMailBox[mbx_code].TIR &= CAN_TI0R_TXRQ;
    if (packet.ext)
    {
        can->sTxMailBox[mbx_code].TIR |= ((packet.id & 0x1FFFFFFF) << 3);
        can->sTxMailBox[mbx_code].TIR |= 1 << 2;
    }
    else
    {
        can->sTxMailBox[mbx_code].TIR |= ((packet.id & 0x7FF) << 21);
    }

    can->sTxMailBox[mbx_code].TIR |= (packet.rtr ? 1 : 0) << 1;

    mailbox->TDTR = (packet.length & 0xF);

    // Reset data registers
    mailbox->TDLR = 0;
    mailbox->TDHR = 0;

    for (uint8_t i = 0; i < packet.length; ++i)
    {
        if (i < 4)
        {
            mailbox->TDLR |= packet.data[i] << i * 8;
        }
        else
        {
            mailbox->TDHR |= packet.data[i] << (i - 4) * 8;
        }
    }

    // Finally send the data
    can->sTxMailBox[mbx_code].TIR |= CAN_TI0R_TXRQ;

    return seq;
}

void Canbus::handleRXInterrupt(int fifo)
{
    CanRXStatus status;
    status.fifo = fifo;
    volatile uint32_t* RFR;
    CAN_FIFOMailBox_TypeDef* mailbox;

    mailbox = &can->sFIFOMailBox[fifo];
    if (fifo == 0)
    {
        RFR = &can->RF0R;
    }
    else
    {
        RFR = &can->RF1R;
    }

    status.fifo_overrun = (*RFR & CAN_RF0R_FOVR0) > 0;
    status.fifo_full    = (*RFR & CAN_RF0R_FULL0) > 0;

    CanPacket p;
    bool hppw = false;

    // Note: Bit position definitions are the same for both FIFOs
    // (CAN_RF0R_FMP0 == CAN_RF1R_FMP1)

    if ((*RFR & CAN_RF0R_FMP0) > 0)
    {
        p.timestamp = miosix::getTick();

        status.rx_status      = *RFR & (CAN_RF0R_FULL0 | CAN_RF0R_FOVR0) >> 3;
        status.err_code       = (can->ESR | CAN_ESR_LEC) >> 4;
        status.rx_err_counter = (can->ESR | CAN_ESR_REC) >> 24;

        p.ext = (mailbox->RIR & CAN_RI0R_IDE) > 0;
        p.rtr = (mailbox->RIR & CAN_RI0R_RTR) > 0;

        if (p.ext)
        {
            p.id = (mailbox->RIR >> 3) & 0x1FFFFFFF;
        }
        else
        {
            p.id = (mailbox->RIR >> 21) & 0x7FF;
        }
        p.length = mailbox->RDTR & CAN_RDT0R_DLC;

        for (uint8_t i = 0; i < p.length; i++)
        {
            if (i < 4)  // Low register
            {
                p.data[i] = (mailbox->RDLR >> (i * 8)) & 0xFF;
            }
            else  // High register
            {
                p.data[i] = (mailbox->RDHR >> ((i - 4) * 8)) & 0xFF;
            }
        }

        *RFR |= CAN_RF0R_RFOM0;

        // Put the message into the queue
        buf_rx_packets.IRQput(CanRXPacket{p, status}, hppw);
    }

    if (hppw)
        miosix::Scheduler::IRQfindNextThread();
}

void Canbus::wakeTXThread()
{
    if (waiting)
    {
        waiting->IRQwakeup();
        waiting = 0;
    }
}

}  // namespace Canbus
}