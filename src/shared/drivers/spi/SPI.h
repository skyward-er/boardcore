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
#include <stddef.h>

#ifndef USE_MOCK_PERIPHERALS
using SPIType = SPI_TypeDef;
#else
#include <utils/testutils/FakeSpiTypedef.h>
using SPIType = FakeSpiTypedef;
#endif

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
 * - 8 master mode baud rate prescales (f_PCLK/2 max.)
 * - Programmable clock polarity and phase
 * - Programmable data order with MSB-first or LSB-first shifting
 * - Hardware CRC feature for reliable communication
 * - DMA capability
 */
class SPI
{
public:
    enum class BitOrder : uint16_t
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
        MODE_0 = 0,  ///< CPOL = 0, CPHA = 0
        MODE_1 = 1,  ///< CPOL = 0, CPHA = 1
        MODE_2 = 2,  ///< CPOL = 1, CPHA = 0
        MODE_3 = 3   ///< CPOL = 1, CPHA = 1
    };

    SPI(SPIType *spi);

    SPIType *getSpi();

    /**
     * @brief Resets the peripheral configuration.
     */
    void reset();

    /**
     * @brief Enables the peripheral.
     */
    void enable();

    /**
     * @brief Disables the peripheral.
     */
    void disable();

    void set8BitFrameFormat();

    void set16BitFrameFormat();

    void enableSoftwareSlaveManagement();

    void disableSoftwareSlaveManagement();

    void enableInternalSlaveSelection();

    void disableInternalSlaveSelection();

    void setBitOrder(BitOrder bitOrder);

    void setClockDiver(ClockDivider divider);

    void setSlaveConfiguration();

    void setMasterConfiguration();

    void setMode(Mode mode);

    void enableTxDMARequest();

    void disableTxDMARequest();

    void enableRxDMARequest();

    void disableRxDMARequest();

    void waitPeripheral();

    // Read, write and transfer operations in master mode

    /**
     * @brief Reads a single byte from the bus.
     *
     * @return Byte read from the bus.
     */
    uint8_t read();

    /**
     * @brief Reads a single half word from the bus.
     *
     * @return Half word read from the bus.
     */
    uint16_t read16();

    /**
     * @brief Reads multiple bytes from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
     */
    void read(uint8_t *data, size_t nBytes);

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer in bytes.
     */
    void read(uint16_t *data, size_t nBytes);

    /**
     * @brief Writes a single byte to the bus.
     *
     * @param data Byte to write.
     */
    void write(uint8_t data);

    /**
     * @brief Writes a single half word to the bus.
     *
     * @param data Half word to write.
     */
    void write(uint16_t data);

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void write(uint8_t *data, size_t nBytes);

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer in bytes.
     */
    void write(uint16_t *data, size_t nBytes);

    /**
     * @brief Full duplex transmission of one byte on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Full duplex transmission of one half word on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer(uint16_t data);

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer in bytes.
     */
    void transfer(uint8_t *data, size_t nBytes);

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer in bytes.
     */
    void transfer(uint16_t *data, size_t nBytes);

private:
    SPIType *spi;
};

inline SPI::SPI(SPIType *spi) : spi(spi) {}

inline SPIType *SPI::getSpi() { return spi; }

inline void SPI::reset()
{
    spi->CR1    = 0;
    spi->CR2    = 0;
    spi->DR     = 0;
    spi->RXCRCR = 0;
    spi->TXCRCR = 0;
}

inline void SPI::enable() { spi->CR1 |= SPI_CR1_SPE; }

inline void SPI::disable() { spi->CR1 &= ~SPI_CR1_SPE; }

inline void SPI::set8BitFrameFormat() { spi->CR1 &= ~SPI_CR1_DFF; }

inline void SPI::set16BitFrameFormat() { spi->CR1 |= SPI_CR1_DFF; }

inline void SPI::enableSoftwareSlaveManagement() { spi->CR1 |= SPI_CR1_SSM; }

inline void SPI::disableSoftwareSlaveManagement() { spi->CR1 &= ~SPI_CR1_SSM; }

inline void SPI::enableInternalSlaveSelection() { spi->CR1 |= SPI_CR1_SSI; }

inline void SPI::disableInternalSlaveSelection() { spi->CR1 &= ~SPI_CR1_SSI; }

inline void SPI::setBitOrder(BitOrder bitOrder)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_LSBFIRST;

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(bitOrder);
}

inline void SPI::setClockDiver(ClockDivider divider)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_BR;

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(divider);
}

inline void SPI::setSlaveConfiguration() { spi->CR1 &= ~SPI_CR1_MSTR; }

inline void SPI::setMasterConfiguration() { spi->CR1 |= SPI_CR1_MSTR; }

inline void SPI::setMode(Mode mode)
{
    // First clear the configuration
    spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(mode);
}

inline void SPI::enableTxDMARequest() { spi->CR2 |= SPI_CR2_TXDMAEN; }

inline void SPI::disableTxDMARequest() { spi->CR2 &= ~SPI_CR2_TXDMAEN; }

inline void SPI::enableRxDMARequest() { spi->CR2 |= SPI_CR2_RXDMAEN; }

inline void SPI::disableRxDMARequest() { spi->CR2 &= ~SPI_CR2_RXDMAEN; }

inline void SPI::waitPeripheral()
{
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while ((spi->SR & SPI_SR_BSY) > 0)
        ;
}

// Read, write and transfer operations in master mode

inline uint8_t SPI::read() { return transfer(static_cast<uint8_t>(0)); }

inline uint16_t SPI::read16() { return transfer(static_cast<uint16_t>(0)); }

inline void SPI::read(uint8_t *data, size_t nBytes)
{
    // Reset the data
    for (size_t i = 0; i < nBytes / 2; i++)
        data[i] = 0;

    // Read the data
    transfer(data, nBytes);
}

inline void SPI::read(uint16_t *data, size_t nBytes)
{
    // Reset the data
    for (size_t i = 0; i < nBytes / 2; i++)
        data[i] = 0;

    // Read the data
    transfer(data, nBytes);
}

inline void SPI::write(uint8_t data)
{
    // Write the data item in the transmit buffer
    spi->DR = data;

    // Wait until Tx buffer is empty and until the peripheral is still busy
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while (spi->SR & SPI_SR_BSY)
        ;

    // Ensures the Rx buffer is empty
    (void)spi->DR;
}

inline void SPI::write(uint16_t data)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    // Write the data item in the Tx buffer
    spi->DR = data;

    // Wait until Tx buffer is empty and until the peripheral is still busy
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while (spi->SR & SPI_SR_BSY)
        ;

    // Go back to 8 bit frame format
    set8BitFrameFormat();

    // Ensures the Rx buffer is empty
    (void)spi->DR;
}

inline void SPI::write(uint8_t *data, size_t nBytes)
{
    // Write the first data item in the Tx buffer
    spi->DR = data[0];

    // Wait for TXE=1 and write the next data item
    for (size_t i = 1; i < nBytes; i++)
    {
        // Wait until Tx buffer is empty
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the next data item
        spi->DR = data[i];
    }

    // Wait until Tx buffer is empty and until the peripheral is still busy
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while (spi->SR & SPI_SR_BSY)
        ;

    // Ensures the Rx buffer is empty
    (void)spi->DR;
}

inline void SPI::write(uint16_t *data, size_t nBytes)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    // Write the first data item in the Tx buffer
    spi->DR = data[0];

    // Wait for TXE=1 and write the next data item
    for (size_t i = 1; i < nBytes / 2; i++)
    {
        // Wait until Tx buffer is empty
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the next data item
        spi->DR = data[i];
    }

    // Wait until Tx buffer is empty and until the peripheral is still busy
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while (spi->SR & SPI_SR_BSY)
        ;

    // Go back to 8 bit frame format
    set8BitFrameFormat();

    // Ensures the Rx buffer is empty
    (void)spi->DR;
}

inline uint8_t SPI::transfer(uint8_t data)
{
    // Write the data item to transmit
    spi->DR = static_cast<uint8_t>(data);

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Read the received data item
    return static_cast<uint8_t>(spi->DR);
}

inline uint16_t SPI::transfer(uint16_t data)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    // Write the data item to transmit
    spi->DR = static_cast<uint16_t>(data);

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Go back to 8 bit frame format
    set8BitFrameFormat();

    // Read the received data item
    return static_cast<uint8_t>(spi->DR);
}

inline void SPI::transfer(uint8_t *data, size_t nBytes)
{
    // Write the first data item to transmit
    spi->DR = data[0];

    for (size_t i = 1; i < nBytes; i++)
    {
        // Wait until the previous data item has been transmitted
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the next data item
        spi->DR = static_cast<uint8_t>(data[i]);

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        data[i - 1] = static_cast<uint8_t>(spi->DR);
    }

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Read the last received data item
    data[nBytes - 1] = static_cast<uint8_t>(spi->DR);
}

inline void SPI::transfer(uint16_t *data, size_t nBytes)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    // Write the first data item to transmit
    spi->DR = data[0];

    for (size_t i = 1; i < nBytes / 2; i++)
    {
        // Wait until the previous data item has been transmitted
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the next data item
        spi->DR = static_cast<uint16_t>(data[i]);

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        data[i - 1] = static_cast<uint16_t>(spi->DR);
    }

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Read the last received data item
    data[nBytes / 2 - 1] = static_cast<uint16_t>(spi->DR);

    // Go back to 8 bit frame format
    set8BitFrameFormat();
}

}  // namespace Boardcore
