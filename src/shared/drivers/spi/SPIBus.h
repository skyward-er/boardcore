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

#include "SPIBusInterface.h"

namespace Boardcore
{

/**
 * @brief Driver for the STM32 low level SPI peripheral.
 *
 * This driver applies to the whole STM32F4xx/STM32F7xx families.
 *
 * The serial peripheral interface (SPI) allows half/full-duplex, synchronous,
 * serial communication with external devices. The interface can be configured
 * as the master and in this case it provides the communication clock (SCK) to
 * the external slave device. The peripheral is also capable of reliable
 * communication using CRC checking.
 *
 * Supported features:
 * - Full-duplex synchronous transfers on three lines
 * - 8-bit transfer frame formats
 * - Master operation
 * - 8 master mode baud rate prescaler values (f_PCLK/2 max.)
 * - Programmable clock polarity and phase
 * - Programmable bit order (MSBit-first or LSBit-first)
 */
class SPIBus : public SPIBusInterface
{
public:
    explicit SPIBus(SPI_TypeDef* spi);

    ///< Delete copy/move contructors/operators.
    SPIBus(const SPIBus&)            = delete;
    SPIBus& operator=(const SPIBus&) = delete;
    SPIBus(SPIBus&&)                 = delete;
    SPIBus& operator=(SPIBus&&)      = delete;

    SPI_TypeDef* getSpi() override;

    void configure(const SPIBusConfig& newConfig) override;

    void transfer(const uint8_t* txData, uint8_t* rxData, size_t size) override;

private:
    /**
     * @brief Full duplex transmission of 8 bits on the bus.
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transferByte(uint8_t data);

    SPI_TypeDef* spi;
    SPIBusConfig config{};
};

}  // namespace Boardcore
