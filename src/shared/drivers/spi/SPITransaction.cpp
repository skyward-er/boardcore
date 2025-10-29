/* Copyright (c) 2019-2025 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Davide Mor, Fabrizio Monti
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

#include "SPITransaction.h"

#include <interfaces/endianness.h>

namespace Boardcore
{

constexpr std::chrono::nanoseconds SPITransaction::defaultTimeout;

SPITransaction::SPITransaction(const SPISlave& slave)
    : slave(slave), spiPtr(slave.bus.getSpi()), dmaTimeout(slave.dmaTimeout)
{
    slave.bus.configure(slave.config);

    useDma = (slave.streamRx != nullptr) && (slave.streamTx != nullptr);
}

SPIBusInterface& SPITransaction::getBus() { return slave.bus; }

void SPITransaction::setDmaTimeout(std::chrono::nanoseconds t)
{
    dmaTimeout = t;
}

void SPITransaction::disableDma() { useDma = false; }

bool SPITransaction::enableDma()
{
    useDma = (slave.streamRx != nullptr) && (slave.streamTx != nullptr);
    return useDma;
}

void SPITransaction::getLastErrors(SPITransactionDMAErrors& txError,
                                   SPITransactionDMAErrors& rxError)
{
    txError = lastErrorTx;
    rxError = lastErrorRx;
}

// Read, write and transfer operations in master mode

uint8_t SPITransaction::read()
{
    slave.bus.select(slave.cs);
    uint8_t data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::read16()
{
    slave.bus.select(slave.cs);
    uint16_t data = slave.bus.read16();
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::read24()
{
    slave.bus.select(slave.cs);
    uint32_t data = slave.bus.read24();
    slave.bus.deselect(slave.cs);
    return data;
}

uint32_t SPITransaction::read32()
{
    slave.bus.select(slave.cs);
    uint32_t data = slave.bus.read32();
    slave.bus.deselect(slave.cs);
    return data;
}

void SPITransaction::read(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::read16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.read16(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write(uint8_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write16(uint16_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write16(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write24(uint32_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write24(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write32(uint32_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write32(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.write16(data, size);
    slave.bus.deselect(slave.cs);
}

uint8_t SPITransaction::transfer(uint8_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::transfer16(uint16_t data)
{
    if (useDma)
    {
        // DMA
        volatile uint8_t sendBuf[]  = {static_cast<uint8_t>(data >> 8),
                                       static_cast<uint8_t>(data)};
        volatile uint8_t recvBuf[2] = {0};

        defaultDmaReceivingSetup((void*)recvBuf, 2);
        defaultDmaTransmittingSetup((void*)sendBuf, 2);

        if (!dmaTransfer(dmaTimeout))
            return 0;

        return recvBuf[0] << 8 | recvBuf[1];
    }

    slave.bus.select(slave.cs);
    data = slave.bus.transfer16(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::transfer24(uint32_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer24(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::transfer32(uint32_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer32(data);
    slave.bus.deselect(slave.cs);

    return data;
}

void SPITransaction::transfer(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.transfer(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::transfer16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.transfer16(data, size);
    slave.bus.deselect(slave.cs);
}

// Read, write and transfer operations with registers

uint8_t SPITransaction::readRegister(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    if (useDma)
    {
        // DMA
        volatile uint8_t dstBuf[]  = {0, 0};
        volatile uint8_t sendBuf[] = {reg, 0};

        defaultDmaReceivingSetup((void*)dstBuf, 2);
        defaultDmaTransmittingSetup((void*)sendBuf, 2);

        if (!dmaTransfer(dmaTimeout))
            return 0;

        return dstBuf[1];
    }

    // Normal
    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint8_t data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::readRegister16(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint16_t data = slave.bus.read16();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes16(data);

    return data;
}

uint32_t SPITransaction::readRegister24(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint32_t data = slave.bus.read24();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes32(data) >> 8;

    return data;
}

uint32_t SPITransaction::readRegister32(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint32_t data = slave.bus.read32();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes32(data) >> 8;

    return data;
}

void SPITransaction::readRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

bool SPITransaction::writeRegister(uint8_t reg, uint8_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    if (useDma)
    {
        // DMA
        volatile uint8_t dstBuf[]  = {0, 0};
        volatile uint8_t sendBuf[] = {reg, data};

        defaultDmaReceivingSetup((void*)dstBuf, 2);
        defaultDmaTransmittingSetup((void*)sendBuf, 2);

        return dmaTransfer(dmaTimeout);
    }

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
    return true;
}

void SPITransaction::writeRegister16(uint8_t reg, uint16_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write16(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister24(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write24(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister32(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write32(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

bool SPITransaction::dmaTransfer(const std::chrono::nanoseconds timeout)
{
    // Disable spi
    spiPtr->CR1 &= ~SPI_CR1_SPE;

    // Start transaction
    slave.bus.select(slave.cs);

    // Enable spi rx buffer dma
    spiPtr->CR2 |= SPI_CR2_RXDMAEN;

    // Enable the receiving stream
    (*slave.streamRx)->enable();

    // Enable the transmitting stream
    (*slave.streamTx)->enable();

    // Enable spi tx buffer dma
    spiPtr->CR2 |= SPI_CR2_TXDMAEN;

    // Enable the spi peripheral
    spiPtr->CR1 |= SPI_CR1_SPE;

    bool resultTransmit =
        (*slave.streamTx)->timedWaitForTransferComplete(timeout);
    bool resultReceive =
        (*slave.streamRx)->timedWaitForTransferComplete(timeout);

    bool spiWaitResult = true;
    if (resultTransmit && resultReceive)
    {
        // DMA completion doesn't guarantee that the SPI peripheral
        // has finished transmitting
        spiWaitResult = spiDmaWaitForTransmissionComplete();
    }

    // Stop the transaction
    slave.bus.deselect(slave.cs);

    // Disable the dma streams
    (*slave.streamTx)->disable();
    (*slave.streamRx)->disable();

    // Disable spi dma transmit and receive
    spiPtr->CR2 &= ~SPI_CR2_TXDMAEN;
    spiPtr->CR2 &= ~SPI_CR2_RXDMAEN;

    // Check for transmitting errors
    if (!resultTransmit)
        lastErrorTx = SPITransactionDMAErrors::DMA_TIMEOUT;
    else if ((*slave.streamTx)->getTransferErrorFlagStatus())
        lastErrorTx = SPITransactionDMAErrors::DMA_TRANSFER_ERROR;
    else if ((*slave.streamTx)->getFifoErrorFlagStatus())
        lastErrorTx = SPITransactionDMAErrors::DMA_FIFO_ERROR;
    else if (!spiWaitResult)
        lastErrorTx = SPITransactionDMAErrors::SPI_TIMEOUT;
    else
        lastErrorTx = SPITransactionDMAErrors::NO_ERRORS;

    // Check for receiving errors
    if (!resultReceive)
        lastErrorRx = SPITransactionDMAErrors::DMA_TIMEOUT;
    else if ((*slave.streamRx)->getTransferErrorFlagStatus())
        lastErrorRx = SPITransactionDMAErrors::DMA_TRANSFER_ERROR;
    else if ((*slave.streamRx)->getFifoErrorFlagStatus())
        lastErrorRx = SPITransactionDMAErrors::DMA_FIFO_ERROR;
    else if (!spiWaitResult)
        lastErrorRx = SPITransactionDMAErrors::SPI_TIMEOUT;
    else
        lastErrorRx = SPITransactionDMAErrors::NO_ERRORS;

    return lastErrorRx == SPITransactionDMAErrors::NO_ERRORS &&
           lastErrorTx == SPITransactionDMAErrors::NO_ERRORS;
}

bool SPITransaction::spiDmaWaitForTransmissionComplete()
{
    // First, ensure the TX buffer is empty, then check the SPI busy
    // flag
    static constexpr int64_t spiDefaultTimeoutNs =
        std::chrono::duration_cast<std::chrono::nanoseconds>(defaultTimeout)
            .count();

    const int64_t timeout = miosix::getTime() + spiDefaultTimeoutNs;

#if defined(STM32F765xx) || defined(STM32F767xx) || defined(STM32F769xx) || \
    defined(STM32F777xx) || defined(STM32F779xx)
    while ((spiPtr->SR & SPI_SR_FTLVL) > 0 && miosix::getTime() < timeout)
    {
    }

    if ((spiPtr->SR & SPI_SR_FTLVL) > 0)
    {
        // Timeout expired
        return false;
    }

#elif defined(STM32F405xx) || defined(STM32F407xx) || defined(STM32F415xx) || \
    defined(STM32F417xx) || defined(STM32F427xx) || defined(STM32F429xx) ||   \
    defined(STM32F437xx) || defined(STM32F439xx)
    while ((spiPtr->SR & SPI_SR_TXE) == 0 && miosix::getTime() < timeout)
    {
    }

    if ((spiPtr->SR & SPI_SR_TXE) == 0)
    {
        // Timeout expired
        return false;
    }
#else
#warning This board is not officially supported. SPITransaction with DMA might not work as expected.
#endif

    while ((spiPtr->SR & SPI_SR_BSY) && miosix::getTime() < timeout)
    {
    }

    if (spiPtr->SR & SPI_SR_BSY)
    {
        // Timeout expired
        return false;
    }

    return true;
}

void SPITransaction::defaultDmaTransmittingSetup(void* srcAddr, uint16_t nBytes)
{
    DMATransaction txSetup;
    defaultDmaSetup(txSetup, DMATransaction::Direction::MEM_TO_PER, srcAddr,
                    (void*)&(spiPtr->DR), nBytes, true, false);
    (*slave.streamTx)->setup(txSetup);
}

void SPITransaction::defaultDmaReceivingSetup(void* dstAddr, uint16_t nBytes)
{
    DMATransaction rxSetup;
    defaultDmaSetup(rxSetup, DMATransaction::Direction::PER_TO_MEM,
                    (void*)&(spiPtr->DR), dstAddr, nBytes, false, true);
    (*slave.streamRx)->setup(rxSetup);
}

void SPITransaction::defaultDmaSetup(DMATransaction& streamSetup,
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
