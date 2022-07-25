/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#pragma once

#include <interfaces-impl/arch_registers_impl.h>
#include <miosix.h>
#include <utils/collections/IRQCircularBuffer.h>

#include "CanDriverData.h"
#include "Filters.h"

using miosix::Thread;

namespace Boardcore
{

namespace Canbus
{

/**
 * @brief Low level CanBus driver, with support for both peripherals (CAN1 and
 * CAN2) on stm32f4 microcontrollers.
 */
class CanbusDriver
{
    // How many frames to store in the RX buffer
    static constexpr unsigned int RX_BUF_SIZE        = 10;
    static constexpr unsigned int TX_STATUS_BUF_SIZE = 10;

    // Weight coefficients for calculating the optimal bit timing
    static constexpr float BR_ERR_WEIGHT = 10;
    static constexpr float SP_ERR_WEIGHT = 1;
    static constexpr float N_ERR_WEIGHT  = 1 / 50;

    static constexpr uint8_t NUM_FILTER_BANKS = 28;

public:
    /**
     * @brief Configuration struct for basic CanBus operation.
     */
    struct CanbusConfig
    {
        uint8_t loopback = 0;  // 1: Loopback mode enabled
        uint8_t awum     = 1;  // Automatic wakeup (1: automatic wakeup upon new
                               // message received)
        uint8_t nart =
            0;  // No auto retransmission (0: packets are retransmitted until
                // success, 1: only one transfer attempt)
        uint8_t abom = 1;  // Automatic bus off management (1: automatically
                           // recovers from bus-off state)
        uint8_t rflm = 1;  // Receive fifo locked (0: new messages overwrite
                           // last ones, 1: new message discarded)
        uint8_t txfp = 1;  // TX Fifo priority (0: identifier, 1: chronological)
    };

    /**
     * @brief Struct defining high level bit timing requirements. Register
     * values will then be calculated automatically.
     */
    struct AutoBitTiming
    {
        /**
         * @brief Canbus baud rate in bps (BITS PER SECOND). CANOpen standard
         * values are preferred but not mandatory: 1000 kbps, 500 kbps, 250
         * kbps, 125 kbps, 100 kbps, 83.333 kbps, 50 kbps, 25 kbps and 10 kbps.
         */
        uint32_t baudRate;

        /**
         * @brief Sample point in percentage of the bit length. Eg: 0.875.
         */
        float samplePoint;
    };

    /**
     * @brief Struct specifying exact bit timing registers values.
     */
    struct BitTiming
    {
        uint16_t BRP;
        uint16_t BS1;
        uint16_t BS2;
        uint8_t SJW;
    };

    /**
     * @brief Construct a new Canbus object, automatically calculating timing
     * register values from high level requirements (baud rate and sample point
     * position).
     *
     * @param can Canbus peripheral pointer (CAN1 or CAN2).
     * @param config Bus configuration.
     * @param bitTiming Bit timing requirements.
     */
    CanbusDriver(CAN_TypeDef* can, CanbusConfig config,
                 AutoBitTiming bitTiming);

    /**
     * @brief Construct a new Canbus object, manually assigning each bit timing
     * register value.
     *
     * Also enables the CAN peripheral clock automatically
     *
     * @param can Canbus peripheral pointer (CAN1 or CAN2).
     * @param config Bus configuration.
     * @param bitTiming Bit timing register values.
     */
    CanbusDriver(CAN_TypeDef* can, CanbusConfig config, BitTiming bitTiming);

    /**
     * @brief Disables the peripheral clock.
     */
    ~CanbusDriver();

    /**
     * @brief Exits initialization mode and starts CanBus operation.
     */
    void init();

    /**
     * @brief Adds a new filter to the bus, or returns false if there are no
     * more filter banks available.
     *
     * @warning Can only be called before init().
     *
     * @param filter Filter to be added
     */
    bool addFilter(FilterBank filter);

    /**
     * @brief Sends a packet on the bus. This function blocks until the message
     * has been successfully put into a TX mailbox.
     *
     * @param packet packet to be send.
     * @return uint32_t Packet sequence number.
     */
    uint32_t send(CanPacket packet);

    /**
     * @brief Returns a reference to the buffer containing received packets.
     */
    IRQCircularBuffer<CanRXPacket, RX_BUF_SIZE>& getRXBuffer()
    {
        return bufRxPackets;
    }

    /**
     * @brief Returns a reference to the buffer containing transfer results and
     * statistics.
     */
    IRQCircularBuffer<CanTXResult, TX_STATUS_BUF_SIZE>& getTXResultBuffer()
    {
        return bufTxResult;
    }

    /**
     * @brief Returns the CanBus peripheral assigned to this instance.
     */
    CAN_TypeDef* getCAN() { return can; }

    /**
     * @brief Gets the sequence number of the message in the ith tx mailbox.
     */
    uint32_t getTXMailboxSequence(uint8_t i) { return txMailboxSeq[i]; }

    /**
     * @brief Wakes the transmission thread. ONLY to be called from the Canbus
     * TX interrupt handler routine.
     */
    void wakeTXThread();

    /**
     * @brief Handles an incoming RX interrupt. ONLY to be called from the
     * Canbus RX interrupt handler routine.
     */
    void handleRXInterrupt(int fifo);

private:
    /**
     * Automatically calculates values for the bit timing register
     * starting from the required baud rate and sample point position, using a
     * simple optimization algorithm to find the closest possible values,
     * prioritizing the minimization of the error in the baud rate.
     */
    static BitTiming calcBitTiming(AutoBitTiming config);

    // Stores RX packets
    IRQCircularBuffer<CanRXPacket, RX_BUF_SIZE> bufRxPackets;

    // Store results of a TX request
    IRQCircularBuffer<CanTXResult, TX_STATUS_BUF_SIZE> bufTxResult;

    CAN_TypeDef* can;

    uint32_t txSeq           = 1;    // TX packet sequence number
    uint32_t txMailboxSeq[3] = {0};  // Seq. no. of packets in the TX mailbox

    uint8_t isInit      = false;
    uint8_t filterIndex = 0;
    Thread* waiting     = nullptr;
};

}  // namespace Canbus

}  // namespace Boardcore
