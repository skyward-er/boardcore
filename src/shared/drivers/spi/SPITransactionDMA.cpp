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

#include "SPITransactionDMA.h"

namespace Boardcore
{

SPITransactionDMA::SPITransactionDMA(const SPISlave& slave, SPIType* ptrSpi,
                                     DMAStreamGuard& rxStream,
                                     DMAStreamGuard& txStream)
    : slave(slave), spi(ptrSpi), streamRx(rxStream), streamTx(txStream)
{
    slave.bus.configure(slave.config);
}

uint8_t SPITransactionDMA::readRegister(uint8_t reg,
                                        std::chrono::microseconds timeout)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    volatile uint8_t dstBuf[]  = {0, 0};
    volatile uint8_t sendBuf[] = {reg, 0};

    DMATransaction trnRx;
    defaultReceivingSetup(trnRx, (void*)dstBuf, 2);
    streamRx->setup(trnRx);

    DMATransaction trnTx;
    defaultTransmittingSetup(trnTx, (void*)sendBuf, 2);
    streamTx->setup(trnTx);

    if (!dmaTransfer(timeout))
        return 0;

    return dstBuf[1];
}

uint16_t SPITransactionDMA::transfer16(uint16_t data,
                                       std::chrono::microseconds timeout)
{
    volatile uint8_t sendBuf[]  = {static_cast<uint8_t>(data >> 8),
                                   static_cast<uint8_t>(data)};
    volatile uint8_t recvBuf[2] = {0};

    DMATransaction trnRx;
    defaultReceivingSetup(trnRx, (void*)recvBuf, 2);
    streamRx->setup(trnRx);

    DMATransaction trnTx;
    defaultTransmittingSetup(trnTx, (void*)sendBuf, 2);
    streamTx->setup(trnTx);

    if (!dmaTransfer(timeout))
        return 0;

    return recvBuf[0] << 8 | recvBuf[1];
}

void SPITransactionDMA::getLastErrors(DMAErrors& txError, DMAErrors& rxError)
{
    txError = lastErrorTx;
    rxError = lastErrorRx;
}

bool SPITransactionDMA::dmaTransfer(const std::chrono::microseconds timeout)
{
    // Disable spi
    spi->CR1 &= ~SPI_CR1_SPE;

    // Start transaction
    slave.bus.select(slave.cs);

    // Enable spi rx buffer dma
    spi->CR2 |= SPI_CR2_RXDMAEN;

    // Enable the receiving stream
    streamRx->enable();

    // Enable the transmitting stream
    streamTx->enable();

    // Enable spi tx buffer dma
    spi->CR2 |= SPI_CR2_TXDMAEN;

    // Enable the spi peripheral
    spi->CR1 |= SPI_CR1_SPE;

    bool resultTransmit = streamTx->timedWaitForTransferComplete(timeout);
    bool resultReceive  = streamRx->timedWaitForTransferComplete(timeout);

    // DMA completion doesn't guarantee that the SPI peripheral
    // has finished transmitting
    spiWaitForTransmissionComplete();

    // Stop the transaction
    slave.bus.deselect(slave.cs);

    // Disable the dma streams
    streamTx->disable();
    streamRx->disable();

    // Disable spi dma transmit and receive
    spi->CR2 &= ~SPI_CR2_TXDMAEN;
    spi->CR2 &= ~SPI_CR2_RXDMAEN;

    // Check for transmitting errors
    if (!resultTransmit)
        lastErrorTx = DMAErrors::TIMEOUT;
    else if (streamTx->getTransferErrorFlagStatus())
        lastErrorTx = DMAErrors::TRANSFER_ERROR;
    else if (streamTx->getFifoErrorFlagStatus())
        lastErrorTx = DMAErrors::FIFO_ERROR;

    // Check for receiving errors
    if (!resultReceive)
        lastErrorRx = DMAErrors::TIMEOUT;
    else if (streamRx->getTransferErrorFlagStatus())
        lastErrorRx = DMAErrors::TRANSFER_ERROR;
    else if (streamRx->getFifoErrorFlagStatus())
        lastErrorRx = DMAErrors::FIFO_ERROR;

    return lastErrorRx == DMAErrors::NO_ERRORS &&
           lastErrorTx == DMAErrors::NO_ERRORS;
}

void SPITransactionDMA::spiWaitForTransmissionComplete()
{
    // First, ensure the TX buffer is empty, then check the SPI busy
    // flag

#ifdef defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F777xx) || defined(STM32F779xx)
    while ((spi->SR & SPI_SR_FTLVL) > 0)
    {
    }
#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) ||   \
    defined(STM32F437xx) || defined(STM32F439xx)
    while ((spi->SR & SPI_SR_TXE) == 0)
    {
    }
#else
#warning This board is not officially supported. SPITransactionDMA might not work as expected.
#endif

    while (spi->SR & SPI_SR_BSY)
    {
    }
}

void SPITransactionDMA::defaultTransmittingSetup(DMATransaction& txSetup,
                                                 void* srcAddr, uint16_t nBytes)
{
    defaultSetup(txSetup, DMATransaction::Direction::MEM_TO_PER, srcAddr,
                 (void*)&(spi->DR), nBytes, true, false);
}

void SPITransactionDMA::defaultReceivingSetup(DMATransaction& rxSetup,
                                              void* dstAddr, uint16_t nBytes)
{
    defaultSetup(rxSetup, DMATransaction::Direction::PER_TO_MEM,
                 (void*)&(spi->DR), dstAddr, nBytes, false, true);
}

void SPITransactionDMA::defaultSetup(DMATransaction& streamSetup,
                                     DMATransaction::Direction dir,
                                     void* srcAddr, void* dstAddr,
                                     uint16_t nBytes, bool srcIncr,
                                     bool dstIncr)
{
    streamSetup = DMATransaction{
        .direction                       = dir,
        .priority                        = DMATransaction::Priority::MEDIUM,
        .srcSize                         = DMATransaction::DataSize::BITS_8,
        .dstSize                         = DMATransaction::DataSize::BITS_8,
        .srcAddress                      = srcAddr,
        .dstAddress                      = dstAddr,
        .secondMemoryAddress             = nullptr,
        .numberOfDataItems               = nBytes,
        .srcIncrement                    = srcIncr,
        .dstIncrement                    = dstIncr,
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
