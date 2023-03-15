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
#include <stdint.h>

namespace Boardcore
{

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
    LSB_FIRST = 0x80
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
    DIV_4   = 0x08,
    DIV_8   = 0x10,
    DIV_16  = 0x18,
    DIV_32  = 0x20,
    DIV_64  = 0x28,
    DIV_128 = 0x30,
    DIV_256 = 0x38,
};

enum class Mode : uint8_t
{
    ///< CPOL = 0, CPHA = 0 -> Clock low when idle, sample on first edge
    MODE_0 = 0,
    ///< CPOL = 0, CPHA = 1 -> Clock low when idle, sample on second edge
    MODE_1 = 1,
    ///< CPOL = 1, CPHA = 0 -> Clock high when idle, sample on first edge
    MODE_2 = 2,
    ///< CPOL = 1, CPHA = 1 -> Clock high when idle, sample on second edge
    MODE_3 = 3
};

enum class WriteBit
{
    NORMAL,    ///< Normal write bit settings (0 for write, 1 for reads)
    INVERTED,  ///< Inverted write bit settings (1 for write, 0 for reads)
    DISABLED,  ///< Do not set write bit in any way
};

}  // namespace SPI

}  // namespace Boardcore
