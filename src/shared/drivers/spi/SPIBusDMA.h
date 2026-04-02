/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <drivers/dma/DMA.h>

#include <chrono>

#include "SPIBus.h"

namespace Boardcore
{

namespace detail
{
// Default max CPU time set to ~2x DMA overhead
constexpr auto DEFAULT_MAX_CPU_TIME = std::chrono::microseconds(20);
}  // namespace detail

/**
 * @brief DMA-enabled driver for the STM32 low level SPI peripheral.
 *
 * This is a DMA-enabled version of SPIBus. It uses the provided DMA streams to
 * perform transfers when the transfer size is above a certain given threshold,
 * while it falls back to the CPU for transfer sizes below the threshold.
 *
 * Supported features are the same as SPIBus, see its documentation for details.
 */
class SPIBusDMA : public SPIBus
{
public:
    /**
     * @brief Construct a new SPIBusDMA object.
     *
     * @param spi SPI peripheral to use.
     * @param txStream DMA stream to use for SPI transmissions, must be valid
     * @param rxStream DMA stream to use for SPI receptions, must be valid
     * @param maxCpuTime Maximum CPU time allowed for a transfer above which the
     * driver should use DMA. A value of 0 always uses DMA.
     */
    SPIBusDMA(
        SPI_TypeDef* spi, DMAStreamGuard&& txStream, DMAStreamGuard&& rxStream,
        std::chrono::microseconds maxCpuTime = detail::DEFAULT_MAX_CPU_TIME);

    ///< Delete copy/move contructors/operators.
    SPIBusDMA(const SPIBusDMA&)            = delete;
    SPIBusDMA& operator=(const SPIBusDMA&) = delete;
    SPIBusDMA(SPIBusDMA&&)                 = delete;
    SPIBusDMA& operator=(SPIBusDMA&&)      = delete;

    void configure(const SPIBusConfig& newConfig) override;

    void transfer(const uint8_t* txData, uint8_t* rxData, size_t size) override;

private:
    /**
     * @brief Determines if a transfer with the provided size is worth
     * performing with DMA or not.
     */
    bool shouldUseDMA(size_t size);

    static DMATransaction makeDMASetup(DMATransaction::Direction dir,
                                       const void* source, void* destination,
                                       uint16_t size, bool sourceIncrement,
                                       bool destinationIncrement);

    size_t dmaThreshold;  ///< Threshold used to decide when to use DMA
    std::chrono::microseconds
        maxByteTime;  ///< Max time allowed to transfer a single byte, used to
                      ///< compute timeouts

    DMAStreamGuard txStream;
    DMAStreamGuard rxStream;
};
}  // namespace Boardcore
