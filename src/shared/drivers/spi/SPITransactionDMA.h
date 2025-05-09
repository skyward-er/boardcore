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
    SPITransactionDMA(const SPISlave& slave, SPIType* ptrSpi,
                      DMAStreamGuard& rxStream, DMAStreamGuard& txStream)
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

        dmaTransfer(wait);

        return dstBuf[1];
    }

    /**
     * @brief Full duplex transmission of 16 bits on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer16(uint16_t data, std::chrono::microseconds wait =
                                           std::chrono::microseconds::zero())
    {
        // uint16_t temp[2] = {0};
        // // Transferring MSB, putting the read value in the MSB of temp
        // temp[0] = static_cast<uint16_t>(transfer(static_cast<uint8_t>(data >>
        // 8)));
        // // Transferring LSB, putting the read value in the LSB of temp
        // temp[1] =
        // static_cast<uint16_t>(transfer(static_cast<uint8_t>(data)));
        // // MSB | LSB
        // return temp[0] << 8 | temp[1];

        uint8_t sendBuf[]  = {static_cast<uint8_t>(data >> 8),
                              static_cast<uint8_t>(data)};
        uint8_t recvBuf[2] = {0};

        DMATransaction trnRx{
            .direction         = DMATransaction::Direction::PER_TO_MEM,
            .priority          = DMATransaction::Priority::VERY_HIGH,
            .srcSize           = DMATransaction::DataSize::BITS_8,
            .dstSize           = DMATransaction::DataSize::BITS_8,
            .srcAddress        = (void*)&(spi->DR),
            .dstAddress        = recvBuf,
            .numberOfDataItems = 2,
            .srcIncrement      = false,
            .dstIncrement      = true,
            .enableTransferCompleteInterrupt = true,
        };
        streamRx->setup(trnRx);

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

        return recvBuf[0] << 8 | recvBuf[1];
    }

private:
    const SPISlave& slave;
    SPIType* spi;
    DMAStreamGuard& streamRx;
    DMAStreamGuard& streamTx;

    /**
     * @brief Perform the dma transaction following the
     * spi rules.
     * @warning The streams must be setup and ready to go.
     */
    bool dmaTransfer(const std::chrono::microseconds wait)
    {
        // Disable spi
        spi->CR1 &= ~SPI_CR1_SPE;

        // Start transaction
        slave.bus.select(slave.cs);

        // enable dma receive dma
        spi->CR2 |= SPI_CR2_RXDMAEN;

        // First enable the receiving stream
        streamRx->enable();

        // Enable sender stream
        streamTx->enable();

        // Enable spi transmit dma
        spi->CR2 |= SPI_CR2_TXDMAEN;

        // Enable spi peripheral
        spi->CR1 |= SPI_CR1_SPE;

        // Wait for the sender to complete before stopping the transaction
        // TODO: make timed wait
        streamTx->waitForTransferComplete();

        // Wait for the receiver to complete
        // TODO: verify the result of the transaction
        streamRx->timedWaitForTransferComplete(wait);

        // Stop the transaction
        slave.bus.deselect(slave.cs);

        // Disable the dma streams
        streamTx->disable();
        streamRx->disable();

        // Disable the spi peripheral
        spi->CR1 &= ~SPI_CR1_SPE;

        // Disable spi dma transmit and receive
        spi->CR2 &= ~SPI_CR2_TXDMAEN;
        spi->CR2 &= ~SPI_CR2_RXDMAEN;

        // TODO: error handling
        return true;
    }
};

}  // namespace Boardcore
