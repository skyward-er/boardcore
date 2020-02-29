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

#include "SPIInterface.h"

#pragma once

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
     * @param    spi       Pointer to the SPI peripheral to be used
     */
    SPIBus(SPI_TypeDef* spi);

    /**
     * @brief See SPIBusInterface::write()
     */
    void write(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::read()
     */
    void read(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::transfer()
     */
    void transfer(uint8_t* data, size_t size) override;

    /**
     * @brief See SPIBusInterface::select()
     */
    void select(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::deselect()
     */
    void deselect(GpioPin& cs) override;

    /**
     * @brief See SPIBusInterface::configure()
     */
    void configure(SPIBusConfig config) override;

private:
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

    SPI_TypeDef* spi;

    SPIBusConfig config;
};

// Defined here and not in the .cpp to make them inline
inline void SPIBus::write(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        write(data + i);
    }
}

inline void SPIBus::read(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        read(data + i);
    }
}

inline void SPIBus::transfer(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        transfer(data + i);
    }
}

inline void SPIBus::select(GpioPin& cs)
{
    cs.low();
    if (config.cs_setup_time_us > 0)
    {
        delayUs(config.cs_setup_time_us);
    }
}

inline void SPIBus::deselect(GpioPin& cs)
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
    spi->DR;
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
    // Write the byte in the transmit buffer
    spi->DR = 0;

    // Wait until byte is transmitted
    while ((spi->SR & SPI_SR_RXNE) == 0)
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}