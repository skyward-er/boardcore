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

#include "SPIBus.h"

namespace Boardcore
{
class SPIBusDMA : public SPIBus
{
public:
    SPIBusDMA(SPI_TypeDef* spi, DMAStreamGuard&& txStream,
              DMAStreamGuard&& rxStream);

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

    DMAStreamGuard txStream;
    DMAStreamGuard rxStream;
};
}  // namespace Boardcore
