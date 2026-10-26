/* Copyright (c) 2020 Skyward Experimental Rocketry
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

/**
 * This header file is inteded to provide all the necessary files needed to use
 * the SPI driver.
 *
 * The driver is divided into 3 levels:
 * - Low: SPIBus is the actual driver that talks directly to the STM32
 * pheripheral
 * - Middle: SPIBusInterface is a common interface that defines which operations
 * the implementation has to privde
 * - High: SPITransaction is a RAII abstraction that manages slave selection
 * through the chip select signal
 */

#include "SPIBus.h"
#include "SPIBusInterface.h"
#include "SPITransaction.h"
#include "board_settings.h"

namespace Boardcore
{
// Small helper function to delay at least a certain number of nanoseconds.
// It Busy-waits for at least `ns` nanoseconds (rounded up to whole CPU cycles).
// Each loop iteration takes at leasts one cycle, so the delay is never
// shorter than requested, but it can be longer. Meant for very short delays,
// such as the minimum chip select high time between two SPI transactions.
template <uint32_t ns>
inline __attribute__((always_inline)) void delayNs()
{
    // Round up to whole clock cycle
    constexpr uint32_t cycles =
        (uint64_t(ns) * (miosix::cpuFrequency / 1000000) + 999) / 1000;

    if constexpr (cycles == 0)
        return;

    uint32_t count = cycles;
    asm volatile(
        "1: subs %[count], %[count], #1 \n"
        "   bne 1b          \n"
        : [count] "+l"(count)
        :
        : "cc", "memory");
}
}  // namespace Boardcore
