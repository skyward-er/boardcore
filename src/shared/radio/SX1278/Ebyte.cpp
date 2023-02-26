/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "Ebyte.h"

namespace Boardcore
{

EbyteFsk::Error EbyteFsk::configure(const Config& config)
{
    // TODO(davide.mor): Validate input parameters
    return SX1278Fsk::configure(config);
}

void EbyteFsk::enableRxFrontend() { rx_enable.high(); }
void EbyteFsk::disableRxFrontend() { rx_enable.low(); }
void EbyteFsk::enableTxFrontend() { tx_enable.high(); }
void EbyteFsk::disableTxFrontend() { tx_enable.low(); }

EbyteLora::Error EbyteLora::configure(const Config& config)
{
    // TODO(davide.mor): Validate input parameters
    return SX1278Lora::configure(config);
}

void EbyteLora::enableRxFrontend() { rx_enable.high(); }
void EbyteLora::disableRxFrontend() { rx_enable.low(); }
void EbyteLora::enableTxFrontend() { tx_enable.high(); }
void EbyteLora::disableTxFrontend() { tx_enable.low(); }

}  // namespace Boardcore
