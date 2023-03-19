/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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

#include <interfaces/delays.h>
#include <utils/ClockUtils.h>

#include "SPIBusInterface.h"

#ifndef USE_MOCK_PERIPHERALS
using SPIType = SPI_TypeDef;
#else
#include <utils/TestUtils/FakeSpiTypedef.h>
using SPIType = Boardcore::FakeSpiTypedef;
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
 * Supported SPI main features:
 * - Full-duplex synchronous transfers on three lines
 * - 8 or 16-bit transfer frame formats
 * - Master operation
 * - 8 master mode baud rate prescaler values (f_PCLK/2 max.)
 * - Programmable clock polarity and phase
 * - Programmable data order (MSBit-first or LSBit-first)
 */
class SPIBus : public SPIBusInterface
{
public:
    SPIBus(SPIType* spi);

    ///< Delete copy/move contructors/operators.
    SPIBus(const SPIBus&)            = delete;
    SPIBus& operator=(const SPIBus&) = delete;
    SPIBus(SPIBus&&)                 = delete;
    SPIBus& operator=(SPIBus&&)      = delete;

    /**
     * @brief Retrieve the pointer to the peripheral currently used.
     */
    SPIType* getSpi();

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

#ifdef _ARCH_CORTEXM7_STM32F7
    void set8bitRXNE();

    void set16bitRXNE();
#endif

    void set8BitFrameFormat();

    void set16BitFrameFormat();

    void enableSoftwareSlaveManagement();

    void disableSoftwareSlaveManagement();

    void enableInternalSlaveSelection();

    void disableInternalSlaveSelection();

    void setBitOrder(SPI::Order bitOrder);

    void setClockDiver(SPI::ClockDivider divider);

    void setSlaveConfiguration();

    void setMasterConfiguration();

    void setMode(SPI::Mode mode);

    void enableTxDMARequest();

    void disableTxDMARequest();

    void enableRxDMARequest();

    void disableRxDMARequest();

    void waitPeripheral();

    /**
     * @brief Configures and enables the bus with the provided configuration.
     *
     * Since this implementation is not synchronized, if configure() is called
     * on an already in use bus nothing will be done.
     *
     * Use SyncedSPIBus if you need to synchronize access to the bus.
     */
    void configure(SPIBusConfig newConfig) override;

    /**
     * @brief See SPIBusInterface::select().
     */
    void select(GpioType cs) override;

    /**
     * @brief See SPIBusInterface::deselect().
     */
    void deselect(GpioType cs) override;

    // Read, write and transfer operations

    /**
     * @brief Reads 8 bits from the bus.
     *
     * @return Byte read from the bus.
     */
    uint8_t read() override;

    /**
     * @brief Reads 16 bits from the bus.
     *
     * @return Half word read from the bus.
     */
    uint16_t read16() override;

    /**
     * @brief Reads 24 bits from the bus.
     *
     * @return Bytes read from the bus (MSB of the uint32_t value will be 0).
     */
    uint32_t read24() override;

    /**
     * @brief Reads 32 bits from the bus.
     *
     * @return Word read from the bus.
     */
    uint32_t read32() override;

    /**
     * @brief Reads multiple bytes from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief Reads multiple half words from the bus
     *
     * @param data Buffer to be filled with received data.
     * @param size Size of the buffer.
     */
    void read16(uint16_t* data, size_t size) override;

    /**
     * @brief Writes 8 bits to the bus.
     *
     * @param data Byte to write.
     */
    void write(uint8_t data) override;

    /**
     * @brief Writes 16 bits to the bus.
     *
     * @param data Half word to write.
     */
    void write16(uint16_t data) override;

    /**
     * @brief Writes 24 bits to the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     */
    void write24(uint32_t data) override;

    /**
     * @brief Writes 32 bits to the bus.
     *
     * @param data Word to write.
     */
    void write32(uint32_t data) override;

    /**
     * @brief Writes multiple bytes to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    void write(const uint8_t* data, size_t size) override;

    /**
     * @brief Writes multiple half words to the bus.
     *
     * @param data Buffer containing data to write.
     * @param size Size of the buffer.
     */
    void write16(const uint16_t* data, size_t size) override;

    /**
     * @brief Full duplex transmission of 8 bits on the bus.
     *
     * @param data Byte to write.
     * @return Byte read from the bus.
     */
    uint8_t transfer(uint8_t data) override;

    /**
     * @brief Full duplex transmission of 16 bits on the bus.
     *
     * @param data Half word to write.
     * @return Half word read from the bus.
     */
    uint16_t transfer16(uint16_t data) override;

    /**
     * @brief Full duplex transmission of 24 bits on the bus.
     *
     * @param data Bytes to write (the MSB of the uint32_t is not used).
     * @return Bytes read from the bus (the MSB of the uint32_t will be 0).
     */
    uint32_t transfer24(uint32_t data) override;

    /**
     * @brief Full duplex transmission of 32 bits on the bus.
     *
     * @param data Word to write.
     * @return Half word read from the bus.
     */
    uint32_t transfer32(uint32_t data) override;

    /**
     * @brief Full duplex transmission of multiple bytes on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief Full duplex transmission of multiple half words on the bus.
     *
     * @param data Buffer containing data to trasfer.
     * @param size Size of the buffer.
     */
    void transfer16(uint16_t* data, size_t size) override;

private:
    SPIType* spi;
    SPIBusConfig config{};
    bool firstConfigApplied = false;
};

inline SPIBus::SPIBus(SPIType* spi) : spi(spi)
{
    ClockUtils::enablePeripheralClock(spi);
}

inline SPIType* SPIBus::getSpi() { return spi; }

inline void SPIBus::reset()
{
    spi->CR1    = 0;
    spi->CR2    = 0;
    spi->DR     = 0;
    spi->RXCRCR = 0;
    spi->TXCRCR = 0;
}

inline void SPIBus::enable() { spi->CR1 |= SPI_CR1_SPE; }

inline void SPIBus::disable() { spi->CR1 &= ~SPI_CR1_SPE; }

#ifndef _ARCH_CORTEXM7_STM32F7

inline void SPIBus::set8BitFrameFormat() { spi->CR1 &= ~SPI_CR1_DFF; }

inline void SPIBus::set16BitFrameFormat() { spi->CR1 |= SPI_CR1_DFF; }

#else

inline void SPIBus::set8bitRXNE() { spi->CR2 |= SPI_CR2_FRXTH; }

inline void SPIBus::set16bitRXNE() { spi->CR2 &= ~SPI_CR2_FRXTH; }

inline void SPIBus::set8BitFrameFormat()
{
    spi->CR2 &= ~SPI_CR2_DS;
    set8bitRXNE();
}

inline void SPIBus::set16BitFrameFormat()
{
    spi->CR2 |= SPI_CR2_DS;
    set16bitRXNE();
}

#endif

inline void SPIBus::enableSoftwareSlaveManagement() { spi->CR1 |= SPI_CR1_SSM; }

inline void SPIBus::disableSoftwareSlaveManagement()
{
    spi->CR1 &= ~SPI_CR1_SSM;
}

inline void SPIBus::enableInternalSlaveSelection() { spi->CR1 |= SPI_CR1_SSI; }

inline void SPIBus::disableInternalSlaveSelection()
{
    spi->CR1 &= ~SPI_CR1_SSI;
}

inline void SPIBus::setBitOrder(SPI::Order bitOrder)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_LSBFIRST;

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(bitOrder);
}

inline void SPIBus::setClockDiver(SPI::ClockDivider divider)
{
    // First clear the configuration
    spi->CR1 &= ~SPI_CR1_BR;

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(divider);
}

inline void SPIBus::setSlaveConfiguration() { spi->CR1 &= ~SPI_CR1_MSTR; }

inline void SPIBus::setMasterConfiguration() { spi->CR1 |= SPI_CR1_MSTR; }

inline void SPIBus::setMode(SPI::Mode mode)
{
    // First clear the configuration
    spi->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);

    // Set the new value
    spi->CR1 |= static_cast<uint32_t>(mode);
}

inline void SPIBus::enableTxDMARequest() { spi->CR2 |= SPI_CR2_TXDMAEN; }

inline void SPIBus::disableTxDMARequest() { spi->CR2 &= ~SPI_CR2_TXDMAEN; }

inline void SPIBus::enableRxDMARequest() { spi->CR2 |= SPI_CR2_RXDMAEN; }

inline void SPIBus::disableRxDMARequest() { spi->CR2 &= ~SPI_CR2_RXDMAEN; }

inline void SPIBus::waitPeripheral()
{
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    while ((spi->SR & SPI_SR_BSY) > 0)
        ;
}

inline void SPIBus::configure(SPIBusConfig newConfig)
{
    // Do not reconfigure if already in the correct configuration.
    if (!firstConfigApplied || newConfig != config)
    {
        // Save the new configuration
        config             = newConfig;
        firstConfigApplied = true;

        // Wait until the peripheral is done before changing configuration
        waitPeripheral();

        // Disable the peripheral
        disable();

        // Configure clock polarity and phase
        setMode(config.mode);

        // Configure clock frequency
        setClockDiver(config.clockDivider);

        // Configure bit order
        setBitOrder(config.bitOrder);

        // Configure chip select and master mode
        enableSoftwareSlaveManagement();
        enableInternalSlaveSelection();
        setMasterConfiguration();

#ifdef _ARCH_CORTEXM7_STM32F7
        // By default we use 8 bit transactions
        spi.set8bitRXNE();
#endif

        // Enable the peripheral
        enable();
    }
}

inline void SPIBus::select(GpioType cs)
{
    cs.low();

    if (config.csSetupTimeUs > 0)
    {
        miosix::delayUs(config.csSetupTimeUs);
    }
}

inline void SPIBus::deselect(GpioType cs)
{
    if (config.csHoldTimeUs > 0)
    {
        miosix::delayUs(config.csHoldTimeUs);
    }

    cs.high();
}

inline uint8_t SPIBus::read() { return transfer(0); }

inline uint16_t SPIBus::read16() { return transfer16(0); }

inline uint32_t SPIBus::read24() { return transfer24(0); }

inline uint32_t SPIBus::read32() { return transfer32(0); }

inline void SPIBus::read(uint8_t* data, size_t nBytes)
{
    for (size_t i = 0; i < nBytes; i++)
        data[i] = read();
}

inline void SPIBus::read16(uint16_t* data, size_t nBytes)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    for (size_t i = 0; i < nBytes / 2; i++)
    {
        // Wait until the peripheral is ready to transmit
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the data item to transmit
        spi->DR = 0;

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        data[i] = static_cast<uint16_t>(spi->DR);
    }

    // Go back to 8 bit frame format
    set8BitFrameFormat();
}

inline void SPIBus::write(uint8_t data) { transfer(data); }

inline void SPIBus::write16(uint16_t data) { transfer16(data); }

inline void SPIBus::write24(uint32_t data) { transfer24(data); }

inline void SPIBus::write32(uint32_t data) { transfer32(data); }

inline void SPIBus::write(const uint8_t* data, size_t nBytes)
{
    for (size_t i = 0; i < nBytes; i++)
        transfer(data[i]);
}

inline void SPIBus::write16(const uint16_t* data, size_t nBytes)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    for (size_t i = 0; i < nBytes / 2; i++)
    {
        // Wait until the peripheral is ready to transmit
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the data item to transmit
        spi->DR = static_cast<uint16_t>(data[i]);

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        (void)spi->DR;
    }

    // Go back to 8 bit frame format
    set8BitFrameFormat();
}

inline uint8_t SPIBus::transfer(uint8_t data)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;

    // Write the data item to transmit
    spi->DR = static_cast<uint8_t>(data);

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Read the received data item
    return static_cast<uint8_t>(spi->DR);
}

inline uint16_t SPIBus::transfer16(uint16_t data)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;

    // Write the data item to transmit
    spi->DR = static_cast<uint16_t>(data);

    // Wait until data is received
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Go back to 8 bit frame format
    set8BitFrameFormat();

    // Read the received data item
    return static_cast<uint16_t>(spi->DR);
}

inline uint32_t SPIBus::transfer24(uint32_t data)
{
    uint32_t res = static_cast<uint32_t>(transfer16(data >> 8)) << 8;
    res |= transfer(data);

    return res;
}

inline uint32_t SPIBus::transfer32(uint32_t data)
{
    uint32_t res = static_cast<uint32_t>(transfer16(data >> 16)) << 16;
    res |= transfer16(data);

    return res;
}

inline void SPIBus::transfer(uint8_t* data, size_t nBytes)
{
    for (size_t i = 0; i < nBytes; i++)
        data[i] = transfer(data[i]);
}

inline void SPIBus::transfer16(uint16_t* data, size_t nBytes)
{
    // Set 16 bit frame format
    set16BitFrameFormat();

    for (size_t i = 0; i < nBytes / 2; i++)
    {
        // Wait until the peripheral is ready to transmit
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;

        // Write the data item to transmit
        spi->DR = static_cast<uint16_t>(data[i]);

        // Wait until data is received
        while ((spi->SR & SPI_SR_RXNE) == 0)
            ;

        // Read the received data item
        data[i] = static_cast<uint16_t>(spi->DR);
    }

    // Go back to 8 bit frame format
    set8BitFrameFormat();
}

}  // namespace Boardcore
