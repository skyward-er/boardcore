/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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
#include <drivers/spi/SPIBusInterface.h>

#include <chrono>

namespace Boardcore
{

class SPITransactionDMA
{
public:
    SPITransactionDMA(const SPISlave& slave, SPIType* ptrSpi, DMAStreamGuard& rxStream,
                      DMAStreamGuard& txStream)
        : slave(slave), spi(ptrSpi), streamRx(rxStream), streamTx(txStream)
    {
    }

    /**
     * @brief Reads an 8 bit register.
     */
    uint8_t readRegister(uint8_t reg, std::chrono::microseconds wait =
                                          std::chrono::microseconds::zero())
    {
        // Configuration taken from the spi driver
        if (slave.config.writeBit == SPI::WriteBit::NORMAL)
            reg |= 0x80;

        // Receiver stream setup
        uint8_t dstBuf[] = {0, 0};
        DMATransaction trnRx{
            .direction         = DMATransaction::Direction::PER_TO_MEM,
            .priority          = DMATransaction::Priority::VERY_HIGH,
            .srcSize           = DMATransaction::DataSize::BITS_8,
            .dstSize           = DMATransaction::DataSize::BITS_8,
            .srcAddress        = (void*)&(spi->DR),
            .dstAddress        = dstBuf,
            .numberOfDataItems = 2,
            .srcIncrement      = false,
            .dstIncrement      = true,
            .enableTransferCompleteInterrupt = true,
        };
        streamRx->setup(trnRx);

        // Sender stream setup
        uint8_t sendBuf[] = {reg, 0};
        DMATransaction trnTx{
            .direction         = DMATransaction::Direction::MEM_TO_PER,
            .priority          = DMATransaction::Priority::VERY_HIGH,
            .srcSize           = DMATransaction::DataSize::BITS_8,
            .dstSize           = DMATransaction::DataSize::BITS_8,
            .srcAddress        = sendBuf,
            .dstAddress        = (void*)&(spi->DR),
            .numberOfDataItems = 2,
            .srcIncrement      = true,
            .dstIncrement      = false,
            .enableTransferCompleteInterrupt = true,
        };
        streamTx->setup(trnTx);

        // Start transaction
        slave.bus.select(slave.cs);

        // First enable the receiving stream
        streamRx->enable();

        // Enable sender stream
        streamTx->enable();

        // Wait for the sender to complete before stopping the transaction
        streamTx->waitForTransferComplete();  // TODO: make timed wait

        // Stop the transaction
        slave.bus.deselect(slave.cs);

        // Wait for the receiver to complete
        // TODO: verify the result of the transaction
        streamRx->timedWaitForTransferComplete(wait);

        return dstBuf[1];
    }

private:
    const SPISlave& slave;
    SPIType* spi;
    DMAStreamGuard& streamRx;
    DMAStreamGuard& streamTx;
};

}  // namespace Boardcore
