/* 
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <cstdint>
#include <cstdio>

namespace Xbee
{
    struct XbeeStatus
    {
        uint8_t tx_retry_count = 0;
        uint8_t tx_delivery_status = 0;
        uint8_t tx_discovery_status = 0;
        unsigned int tx_timeout_count = 0;

        unsigned int rx_dropped_buffers = 0;
        unsigned int rx_wrong_checksum = 0;

        // void print()
        // {
        //     printf("+++XBEE STATUS+++");
        //     printf("TX: Timeouts: %d\n", tx_timeout_count);
        //     printf("TXSTATUS: Retries: %d, Delivery: %02X, Discovery: %02X\n",
        //            tx_retry_count, tx_delivery_status, tx_discovery_status);
        //     printf("RX: Dropped: %d, Checksum: %d\n", rx_dropped_buffers, rx_wrong_checksum);
                
        // }
    };
}