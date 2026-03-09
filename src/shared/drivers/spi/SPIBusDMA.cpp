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

#include "SPIBusDMA.h"

namespace Boardcore
{
SPIBusDMA::SPIBusDMA(SPI_TypeDef* spi, DMAStreamGuard&& txStream,
                     DMAStreamGuard&& rxStream,
                     std::chrono::microseconds maxCpuTime)
    : SPIBus(spi), txStream(std::move(txStream)), rxStream(std::move(rxStream))
{
    // Compute the dma threshold based on the current APB bus clock speed
    auto apb           = ClockUtils::getPeripheralBus(spi);
    uint32_t apbClock  = ClockUtils::getAPBPeripheralsClock(apb);
    uint32_t byteSpeed = apbClock / 8;

    dmaThreshold = maxCpuTime.count() * byteSpeed / 1000000;
}

void SPIBusDMA::configure(const SPIBusConfig& newConfig)
{
    if (newConfig == config)
        return;

    config = newConfig;

    // Wait until the peripheral is done before changing configuration
    SPI::WaitNotBusy(spi);
    SPI::Disable(spi);

    // Configure clock polarity and phase
    SPI::SetMode(spi, config.mode);
    // Configure clock frequency
    SPI::SetClockDiver(spi, config.clockDivider);
    // Configure bit order
    SPI::SetBitOrder(spi, config.bitOrder);

    // Skip enabling, it will always be done later in transfer()
}

void SPIBusDMA::transfer(const uint8_t* txData, uint8_t* rxData, size_t size)
{
    if (!shouldUseDMA(size))
    {
        SPI::Disable(spi);
        SPI::DisableDMA(spi);
        SPI::Enable(spi);

        SPIBus::transfer(txData, rxData, size);
        return;
    }

    volatile uint8_t txDummy = 0;
    volatile uint8_t rxSink  = 0;
    bool txIncrement         = true;
    bool rxIncrement         = true;

    if (txData == nullptr)
    {
        txData      = (uint8_t*)&txDummy;
        txIncrement = false;
    }

    if (rxData == nullptr)
    {
        rxData      = (uint8_t*)&rxSink;
        rxIncrement = false;
    }

    auto txConfig = makeDMASetup(DMATransaction::Direction::MEM_TO_PER, txData,
                                 (void*)&spi->DR, size, txIncrement, false);
    txStream->setup(txConfig);

    auto rxConfig =
        makeDMASetup(DMATransaction::Direction::PER_TO_MEM, (void*)&spi->DR,
                     rxData, size, false, rxIncrement);
    rxStream->setup(rxConfig);

    // Enable DMA streams and DMA request generation
    SPI::Disable(spi);
    SPI::EnableDMA(spi);
    rxStream->enable();
    txStream->enable();
    SPI::Enable(spi);  // <-- Transfer starts here

    rxStream->timedWaitForTransferComplete(std::chrono::milliseconds(100));

    // RX DMA completion should guarantee that the SPI peripheral is done
    // Check the peripheral anyway to avoid interrupting transactions
    SPI::WaitNotBusy(spi);

    // Disable the dma streams
    txStream->disable();
    rxStream->disable();
}

bool SPIBusDMA::shouldUseDMA(size_t size)
{
    // Because of the fixed overhead of setting up the DMA transfer and the
    // context switch, for small transfers it is faster to just use the CPU for
    // the transfer instead of using DMA.
    //
    // We introduce a WORKLOAD metric as the product of the size in bytes and
    // the clock divider (size * divider). This is used in heuristic evaluation
    // of whether using DMA is worth it or not for a transfer of a given size at
    // the current clock divider.

    // config.clockDivider uses the STM32 register bit-pattern (0, 8, 16... 56)
    // Every increment of 8 in the pattern represents a doubling of the divider
    // (2, 4, 8... 256). We want to map the bit-pattern to the actual
    // divider value without using a switch or a lookup table, we do so by
    // shifting the bit-pattern right by 3 (dividing by 8) and adding 1.
    // This gives us the exponent to which 2 must be raised to get the divider
    // value.
    // e.g. DIV_16 has bit-pattern 0x18 (0x18 = 24 = 0b11000)
    // (0b11000 >> 3) + 1 = 0b11 + 1 = 3 + 1 = 4 -> 2^4 = 16
    int shift = (static_cast<unsigned int>(config.clockDivider) >> 3) + 1;

    // workload = size in bytes * clock divider
    // By shifting 'size' left by 'shift', we effectively compute
    // (size * 2^shift), which is (size * divider)
    size_t workload = size << shift;

    return workload >= dmaThreshold;
}

DMATransaction SPIBusDMA::makeDMASetup(DMATransaction::Direction dir,
                                       const void* source, void* destination,
                                       uint16_t size, bool sourceIncrement,
                                       bool destinationIncrement)
{
    return DMATransaction{
        .direction                       = dir,
        .priority                        = DMATransaction::Priority::MEDIUM,
        .srcSize                         = DMATransaction::DataSize::BITS_8,
        .dstSize                         = DMATransaction::DataSize::BITS_8,
        .srcAddress                      = source,
        .dstAddress                      = destination,
        .secondMemoryAddress             = nullptr,
        .numberOfDataItems               = size,
        .srcIncrement                    = sourceIncrement,
        .dstIncrement                    = destinationIncrement,
        .circularMode                    = false,
        .doubleBufferMode                = false,
        .enableTransferCompleteInterrupt = true,
        .enableHalfTransferInterrupt     = false,
        .enableTransferErrorInterrupt    = true,
        .enableFifoErrorInterrupt        = true,
        .enableDirectModeErrorInterrupt  = false,
    };
}

}  // namespace Boardcore
