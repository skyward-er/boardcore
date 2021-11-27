/* Copyright (c) 2021 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <miosix.h>

#include "CanData.h"
#include "Filters.h"
#include "utils/collections/IRQCircularBuffer.h"

using miosix::Thread;

namespace Boardcore
{
namespace Canbus
{
class Canbus
{
    static constexpr unsigned int RX_BUF_SIZE     = 10;
    static constexpr unsigned int STATUS_BUF_SIZE = 10;

    // Weight coefficients for calculating the optimal bit timing
    static constexpr float BR_ERR_WEIGHT = 10;
    static constexpr float SP_ERR_WEIGHT = 1;
    static constexpr float N_ERR_WEIGHT  = 1 / 50;
    
    static constexpr uint8_t NUM_FILTER_BANKS = 28;

public:
    struct CanbusConfig
    {
        uint8_t loopback = 0;  // 1: Loopback mode enabled
        uint8_t awum     = 1;  // Automatic wakeup (1: automatic wakeup upon new
                               // message received)
        uint8_t nart =
            0;  // No auto retransmission (0: packets are retrasmitted until
                // success, 1: only one transfer attempt)
        uint8_t abom = 1;  // Automatic bus off management (1: automatically
                           // recovers from bus-off state)
        uint8_t rflm = 1;  // Receive fifo locked (0: new messages overwrite
                           // last ones, 1: new message discarded)
        uint8_t txfp = 0;  // TX Fifo priority (0: identifier, 1: chronological)
    };

    /**
     * @brief Struct defining bit timing requirements
     */
    struct AutoBitTiming
    {
        /**
         * @brief Canbus baud rate in bps. CANOpen standard values are preferred
         * but not mandatory: 1000 kpbs, 500 kbps, 250 kbps, 125 kbps, 100
         * kbps, 83.333 kbps, 50 kbps, 25 kbps and 10 kbps
         */
        uint32_t baud_rate;

        /**
         * @brief Sample point in percentage of the bit length. Eg: 0.875
         */
        float sample_point;
    };

    /**
     * @brief Struct specifing exact bit timing registers values
     */
    struct BitTiming
    {
        uint16_t BRP;
        uint16_t BS1;
        uint16_t BS2;
        uint8_t SJW;
    };

    Canbus(CAN_TypeDef* can, CanbusConfig config, AutoBitTiming bit_timing);
    Canbus(CAN_TypeDef* can, CanbusConfig config, BitTiming bit_timing);

    bool addFilter(Filter filter);
    void init();

    uint32_t send(CanPacket packet);

    void handleRXInterrupt(int fifo);
    void wakeTXThread();

    uint32_t getTXMailboxSequence(uint8_t i) { return tx_mailbox_seq[i]; }

    IRQCircularBuffer<CanRXPacket, RX_BUF_SIZE>& getRXBuffer()
    {
        return buf_rx_packets;
    }

    IRQCircularBuffer<CanTXResult, STATUS_BUF_SIZE>& getTXResultBuffer()
    {
        return buf_tx_result;
    }

    CAN_TypeDef* getCAN() { return can; }

private:
    static BitTiming calcBitTiming(AutoBitTiming config);

    // Stores RX packets
    IRQCircularBuffer<CanRXPacket, RX_BUF_SIZE> buf_rx_packets;

    // Store results of a TX request
    IRQCircularBuffer<CanTXResult, STATUS_BUF_SIZE> buf_tx_result;

    CAN_TypeDef* can;

    uint32_t tx_seq = 1; // TX packet sequence number
    uint32_t tx_mailbox_seq[3]; // Sequence number of the packets in the TX mailbox

    uint8_t is_init      = false;
    uint8_t filter_index = 0;
    Thread* waiting    = nullptr;
};
}  // namespace Canbus
}  // namespace Boardcore