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
     * @param rxStream Dma receiving stream for the spi bus.
     * @param txStream Dma transmitting stream for the spi bus.
     */
    SPITransactionDMA(const SPISlave& slave, DMAStreamGuard& rxStream,
                      DMAStreamGuard& txStream);

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
    uint8_t readRegister(uint8_t reg, std::chrono::nanoseconds timeout =
                                          std::chrono::nanoseconds::zero());

    /**
     * @brief Full duplex transmission of 16 bits on the bus.
     *
     * @param data Half word to write.
     * @param timeout The maximum time that will be waited, defaults to waiting
     * forever.
     * @return Half word read from the bus, 0 in case of errors.
     */
    uint16_t transfer16(uint16_t data, std::chrono::nanoseconds timeout =
                                           std::chrono::nanoseconds::zero());

    /**
     * @brief Get last errors for debugging purposes. Avoid silent fails.
     * @param txError Variable where the transmitting error will be stored.
     * @param rxError Variable where the receiving error will be stored.
     */
    // void getLastErrors(SPITransactionDMAErrors& txError,
    //                    SPITransactionDMAErrors& rxError);

private:
    const SPISlave& slave;
};

}  // namespace Boardcore
