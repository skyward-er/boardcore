/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <cassert>

#include "SPIBusInterface.h"


#ifndef USE_MOCK_PERIPHERALS
using SPIType = SPI_TypeDef;
#else
#include "test/FakeSpiTypedef.h"
using SPIType = FakeSpiTypedef;
#endif

/**
 * @brief Low level driver for communicating on a SPI Bus, provides
 *        SPIBusInterface.
 */
class SPIBus : public SPIBusInterface
{
public:
    /**
     * @brief Instantiates a new SPIBus
     *
     * @param spi Pointer to the SPI peripheral to be used
     */
    SPIBus(SPIType* spi) : spi(spi) {}
    ~SPIBus() {}

    // Delete copy/move contructors/operators
    SPIBus(const SPIBus&) = delete;
    SPIBus& operator=(const SPIBus&) = delete;

    SPIBus(SPIBus&&) = delete;
    SPIBus& operator=(SPIBus&&) = delete;

    /**
     * @brief Disable bus configuration: calls to configure() will have no
     * effect after calling this & the SPI peripheral will need to be configured
     * manually.
     */
    void disableBusConfiguration() { config_enabled = false; }

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t byte) override;

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::read()
     */
    uint8_t read() override;

    /**
     * @brief See SPIBusInterface::read()
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    uint8_t transfer(uint8_t data) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::select()
     */
    void select(GpioType& cs) override;

    /**
     * @brief See SPIBusInterface::deselect()
     */
    void deselect(GpioType& cs) override;

    /**
     * @brief Obtains ownership of the bus, configuring it with the provided
     * config. Since this implementation is not syncronized, if acquire() is
     * called on an already locked bus, it will:
     * - FAIL in DEBUG mode
     * - Do nothing when NOT in DEBUG mode
     *
     * Use SyncedSPIBus if you need to synchronize access to the bus.
     */
    void acquire(SPIBusConfig config) override;

protected:
    /**
     * Writes a single byte on the SPI bus.
     *
     * @param    byte Pointer to the byte to be written
     */
    void write(uint8_t* byte);

    /**
     * Reads a single byte from the SPI bus.
     *
     * @param    byte Pointer to the byte where the read data will be stored
     */
    void read(uint8_t* byte);

    /**
     * Full duplex transfer. Writes a single byte on the SPI bus and replaces
     * its content with the received data
     *
     * @param    byte Pointer to the byte to be transfered
     */
    void transfer(uint8_t* byte);

    void configure(SPIBusConfig new_config);

    SPIType* spi;

    SPIBusConfig config{};
    bool config_enabled       = true;
    bool first_config_applied = false;
};

// Defined here and not in the .cpp to make them inline

inline void SPIBus::write(uint8_t data) { write(&data); }

inline void SPIBus::write(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        write(data + i);
    }
}

inline uint8_t SPIBus::read()
{
    uint8_t data;
    read(&data);

    return data;
}

inline void SPIBus::read(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        read(data + i);
    }
}

inline uint8_t SPIBus::transfer(uint8_t data)
{
    transfer(&data);
    return data;
}

inline void SPIBus::transfer(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; ++i)
    {
        transfer(data + i);
    }
}

inline void SPIBus::select(GpioType& cs)
{
    cs.low();
    if (config.cs_setup_time_us > 0)
    {
        delayUs(config.cs_setup_time_us);
    }
}

inline void SPIBus::deselect(GpioType& cs)
{
    if (config.cs_hold_time_us > 0)
    {
        delayUs(config.cs_hold_time_us);
    }
    cs.high();
}

inline void SPIBus::write(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write the byte in the transmit buffer
    spi->DR = *byte;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Clear the RX buffer by accessing the DR register
    (void)spi->DR;
}

inline void SPIBus::transfer(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write the byte in the transmit buffer
    spi->DR = *byte;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}

inline void SPIBus::read(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while ((spi->SR & SPI_SR_TXE) == 0)
        ;
    // Write 0 in the transmit buffer
    spi->DR = 0;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}

inline void SPIBus::acquire(SPIBusConfig new_config)
{
    // Assert that the bus is not already acquired.
    // This bus is not syncronized: fail if someone tries to take ownership when
    // the bus is already being used. Use SyncedSPIBus if you need to
    // synchronize access to the bus
#ifdef DEBUG
    assert(isBusy() == false);
#endif

    SPIBusInterface::acquire(new_config);

    configure(new_config);
}

inline void SPIBus::configure(SPIBusConfig new_config)
{
    // Reconfigure the bus only if config enabled. Do not reconfigure if already
    // in the correct configuration.
    if (config_enabled && (!first_config_applied || new_config != config))
    {
        first_config_applied = true;
        config               = new_config;

        // Wait until the peripheral is done before changing configuration
        while ((spi->SR & SPI_SR_TXE) == 0)
            ;
        while ((spi->SR & SPI_SR_BSY) > 0)
            ;

        spi->CR1 = 0;

        // Configure CPOL & CPHA bits
        spi->CR1 |= static_cast<uint32_t>(config.mode);

        // Configure clock division (BR bits)
        spi->CR1 |= static_cast<uint32_t>(config.clock_div);

        // Configure LSBFIRST bit
        spi->CR1 |= static_cast<uint32_t>(config.bit_order);

        spi->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM  // Use software chip-select
                    | SPI_CR1_MSTR             // Master mode
                    | SPI_CR1_SPE;             // Enable SPI
    }
}