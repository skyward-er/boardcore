/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <interfaces/arch_registers.h>

#include <cstdint>

namespace Boardcore
{

#ifndef USE_MOCK_PERIPHERALS
using GpioType = miosix::GpioPin;
#else
#include <utils/TestUtils/MockGpioPin.h>
using GpioType = Boardcore::MockGpioPin;
#endif

/**
 * @brief Driver for STM32 low level SPI peripheral.
 *
 * This driver applies to the whole STM32F4xx family.
 *
 * The serial peripheral interface (SPI) allows half/full-duplex, synchronous,
 * serial communication with external devices. The interface can be configured
 * as the master and in this case it provides the communication clock (SCK) to
 * the external slave device. The peripheral is also capable of reliable
 * communication using CRC checking.
 *
 * SPI main features:
 * - Full-duplex synchronous transfers on three lines
 * - 8- or 16-bit transfer frame format selection
 * - Master or slave operation
 * - 8 master mode baud rate prescaler (f_PCLK/2 max.)
 * - Programmable clock polarity and phase
 * - Programmable data order with MSB-first or LSB-first shifting
 * - Hardware CRC feature for reliable communication
 * - DMA capability
 */
namespace SPI
{

enum class Order : uint16_t
{
    MSB_FIRST = 0,
    LSB_FIRST = SPI_CR1_LSBFIRST
};

/**
 * @brief SPI Clock divider.
 *
 * SPI clock frequency will be equal to the SPI peripheral bus clock speed
 * divided by the specified value.
 *
 * Eg: DIV_2 --> spi clock freq = f_PCLK / 2
 */
enum class ClockDivider : uint8_t
{
    DIV_2   = 0x00,
    DIV_4   = SPI_CR1_BR_0,
    DIV_8   = SPI_CR1_BR_1,
    DIV_16  = SPI_CR1_BR_1 | SPI_CR1_BR_0,
    DIV_32  = SPI_CR1_BR_2,
    DIV_64  = SPI_CR1_BR_2 | SPI_CR1_BR_0,
    DIV_128 = SPI_CR1_BR_2 | SPI_CR1_BR_1,
    DIV_256 = SPI_CR1_BR
};

enum class Mode : uint8_t
{
    ///< CPOL = 0, CPHA = 0 -> Clock low when idle, sample on first edge
    MODE_0 = 0,
    ///< CPOL = 0, CPHA = 1 -> Clock low when idle, sample on second edge
    MODE_1 = SPI_CR1_CPHA,
    ///< CPOL = 1, CPHA = 0 -> Clock high when idle, sample on first edge
    MODE_2 = SPI_CR1_CPOL,
    ///< CPOL = 1, CPHA = 1 -> Clock high when idle, sample on second edge
    MODE_3 = SPI_CR1_CPOL | SPI_CR1_CPHA
};

enum class WriteBit
{
    NORMAL,    ///< Normal write bit settings (0 for write, 1 for reads)
    INVERTED,  ///< Inverted write bit settings (1 for write, 0 for reads)
    DISABLED,  ///< Do not set write bit in any way
};

inline void Reset(SPI_TypeDef* spi)
{
    spi->CR1    = 0x0000;  // NOTE: this also disables the peripheral
    spi->CR2    = 0x0700;
    spi->CRCPR  = 0x0007;
    spi->RXCRCR = 0x0000;
    spi->TXCRCR = 0x0000;
}

// clang-format off
inline void Enable(SPI_TypeDef* spi) { spi->CR1 |= SPI_CR1_SPE; }
inline void Disable(SPI_TypeDef* spi) { spi->CR1 &= ~SPI_CR1_SPE; }
inline void EnableDMA(SPI_TypeDef* spi) { spi->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN; }
inline void DisableDMA(SPI_TypeDef* spi) { spi->CR2 &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN); }

inline void EnableSoftwareSlaveManagement(SPI_TypeDef* spi) { spi->CR1 |= SPI_CR1_SSM; }
inline void DisableSoftwareSlaveManagement(SPI_TypeDef* spi) { spi->CR1 &= ~SPI_CR1_SSM; }
inline void EnableInternalSlaveSelect(SPI_TypeDef* spi) { spi->CR1 |= SPI_CR1_SSI; }
inline void DisableInternalSlaveSelect(SPI_TypeDef* spi) { spi->CR1 &= ~SPI_CR1_SSI; }
inline void SetSlaveConfiguration(SPI_TypeDef* spi) { spi->CR1 &= ~SPI_CR1_MSTR; }
inline void SetMasterConfiguration(SPI_TypeDef* spi) { spi->CR1 |= SPI_CR1_MSTR; }

inline void EnableTxDMA(SPI_TypeDef* spi) { spi->CR2 |= SPI_CR2_TXDMAEN; }
inline void EnableRxDMA(SPI_TypeDef* spi) { spi->CR2 |= SPI_CR2_RXDMAEN; }
inline void DisableTxDMA(SPI_TypeDef* spi) { spi->CR2 &= ~SPI_CR2_TXDMAEN; }
inline void DisableRxDMA(SPI_TypeDef* spi) { spi->CR2 &= ~SPI_CR2_RXDMAEN; }
// clang-format on

#ifdef _ARCH_CORTEXM7_STM32F7
/**
 * The SPI peripheral differs on stm32f7 microcontrollers. Refer to AN4660 for a
 * comprehensive differences list between different peripherals versions.
 *
 * The main difference here is that on the f7 you can transmit between 4 and 16.
 * There is also a 32bit fifo and a threshold that generates the RXNE event.
 * For this reason, on f7s we need to configure the 16 bit frame format
 * differently and change the fifo threshold level.
 */

inline void Set8bitRXNE(SPI_TypeDef* spi) { spi->CR2 |= SPI_CR2_FRXTH; }
inline void Set8bitFrameFormat(SPI_TypeDef* spi)
{
    // Set data size to 8 bit
    spi->CR2 &= ~SPI_CR2_DS;
    Set8bitRXNE(spi);
}
#else
inline void Set8bitFrameFormat(SPI_TypeDef* spi) { spi->CR1 &= ~SPI_CR1_DFF; }
#endif

inline void SetBitOrder(SPI_TypeDef* spi, SPI::Order bitOrder)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_LSBFIRST;
    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(bitOrder);
}

inline void SetClockDiver(SPI_TypeDef* spi, SPI::ClockDivider divider)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_BR;
    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(divider);
}

inline void SetMode(SPI_TypeDef* spi, SPI::Mode mode)
{
    // First clear the configuration
    spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(mode);
}

inline void WaitNotBusy(SPI_TypeDef* spi)
{
    while ((spi->SR & SPI_SR_BSY) > 0)
        ;
}

inline void WaitTxEmpty(SPI_TypeDef* spi)
{
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
}

inline void WaitRxNotEmpty(SPI_TypeDef* spi)
{
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;
}

inline void FlushRx(SPI_TypeDef* spi)
{
    while ((spi->SR & SPI_SR_RXNE) != 0)
        spi->DR;
}

}  // namespace SPI
}  // namespace Boardcore
