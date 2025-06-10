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
#include <drivers/spi/SPIBus.h>
#include <drivers/spi/SPIBusInterface.h>

#include <chrono>

namespace Boardcore
{

/**
 * @brief Provides high-level access to the SPI Bus for a single spi
 * transaction with dma.
 *
 * To make sure the bus is properly configured for the provided slave, you have
 * to create a new instance of this class for every transaction, as the bus is
 * configured upon instantiation.
 *
 * @warning DO NOT store an instance of this class for later use, as the bus and
 * the dma streams may be incorrectly configured by then.
 */
class SPITransactionDMA
{
public:
    /**
     * @param slave Slave to communicate with.
     * @param ptrSpi Pointer to the spi peripheral.
     * @param rxStream Dma receiving stream for the spi bus.
     * @param txStream Dma transmitting stream for the spi bus.
     */
    SPITransactionDMA(const SPISlave& slave, SPIType* ptrSpi,
                      DMAStreamGuard& rxStream, DMAStreamGuard& txStream);

    // Delete copy/move constructors/operators
    SPITransactionDMA(const SPITransactionDMA&)            = delete;
    SPITransactionDMA& operator=(const SPITransactionDMA&) = delete;
    SPITransactionDMA(SPITransactionDMA&&)                 = delete;
    SPITransactionDMA& operator=(SPITransactionDMA&&)      = delete;

    /**
     * @brief Reads an 8 bit register.
     * @param reg Register address.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return Byte read from the register, 0 in case of errors.
     */
    uint8_t readRegister(uint8_t reg, std::chrono::microseconds timeout =
                                          std::chrono::microseconds::zero());

    /**
     * @brief Full duplex transmission of 16 bits on the bus.
     *
     * @param data Half word to write.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return Half word read from the bus, 0 in case of errors.
     */
    uint16_t transfer16(uint16_t data, std::chrono::microseconds timeout =
                                           std::chrono::microseconds::zero());

    /**
     * @brief Get last errors for debugging purposes. Avoid silent fails.
     * @param txError Variable where the dma transmitting error will be stored.
     * @param rxError Variable where the dma receiving error will be stored.
     */
    void getLastErrors(DMAErrors& txError, DMAErrors& rxError);

private:
    const SPISlave& slave;
    SPIType* spi;
    DMAStreamGuard& streamRx;
    DMAStreamGuard& streamTx;
    // Last error for the transmitting stream
    DMAErrors lastErrorTx = DMAErrors::NO_ERRORS;
    // Last error for the receiving stream
    DMAErrors lastErrorRx = DMAErrors::NO_ERRORS;

    /**
     * @brief Perform the dma transaction.
     * @warning The streams must be setup and ready to go.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return True if the operation was successful. False otherwise, sets
     * the last error.
     */
    bool dmaTransfer(const std::chrono::microseconds timeout);

    /**
     * @brief Setup the configuration struct with the default sender values
     * needed for an spi transaction.
     * @param txSetup The struct to be configured.
     * @param srcAddr Source address.
     * @param nBytes Number of bytes to be transmitted.
     */
    void defaultTransmittingSetup(DMATransaction& txSetup, void* srcAddr,
                                  uint16_t nBytes);

    /**
     * @brief Setup the configuration struct with the default receiver values
     * needed for an spi transaction.
     * @param rxSetup The struct to be configured.
     * @param dstAddr Destination address.
     * @param nBytes Number of bytes to be received.
     */
    void defaultReceivingSetup(DMATransaction& rxSetup, void* dstAddr,
                               uint16_t nBytes);

    /**
     * @brief Setup the configuration struct with the default values needed
     * for an spi transaction.
     * @param streamSetup The struct to be configured.
     * @param dir Direction of the transaction.
     * @param srcAddr Source address.
     * @param dstAddr Destination address.
     * @param nBytes Number of bytes to be transmitted/received.
     * @param srcIncr Flag, true if the source address must be incremented.
     * @param dstIncr Flag, true if the destination address must be incremented.
     */
    void defaultSetup(DMATransaction& streamSetup,
                      DMATransaction::Direction dir, void* srcAddr,
                      void* dstAddr, uint16_t nBytes, bool srcIncr,
                      bool dstIncr);
};

}  // namespace Boardcore
