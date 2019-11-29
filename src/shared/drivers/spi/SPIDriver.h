/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <miosix.h>
#include <cstdint>

using miosix::delayUs;
using miosix::GpioPin;

struct SPIBusConfig
{
    uint8_t clock_division = 0;
    bool cpol              = 0;
    bool cpha              = 0;
    bool lsb_first         = false;

    // How long to wait after cs is asserted before starting a transmission
    // (microsecodns)
    unsigned int cs_setup_time_us = 0;

    // How long to hold the cs low after the end of the transmission
    // (microseconds)
    unsigned int cs_hold_time_us = 0;
};

class SPIBusInterface
{
public:
    virtual void write(uint8_t* data, size_t size) = 0;
    virtual void read(uint8_t* data, size_t size)  = 0;

    virtual void select(GpioPin cs)   = 0;
    virtual void deselect(GpioPin cs) = 0;

    virtual void configure(SPIBusConfig config) = 0;
};

class SPIBus : public SPIBusInterface
{
public:
    SPIBus(SPI_TypeDef* spi, uint32_t* rcc_en_reg, uint32_t rcc_spi_en_bit);

    void write(uint8_t* data, size_t size) override;
    void read(uint8_t* data, size_t sizes) override;

    void select(GpioPin cs) override;
    void deselect(GpioPin cs) override;

    void configure(SPIBusConfig config) override;

private:
    void enablePeripheral();

    void transfer(uint8_t* byte);
    void read(uint8_t* byte);

    SPI_TypeDef* spi;
    uint32_t* rcc_en_reg;
    uint32_t rcc_spi_en_bit;

    SPIBusConfig last_config;
};

struct SPISlave
{
    SPIBusInterface& bus;

    SPIBusConfig config;
    GpioPin cs;

    SPISlave(SPIBusInterface& bus, SPIBusConfig config, GpioPin cs)
        : bus(bus), config(config), cs(cs)
    {
    }
};

class SPITransaction
{
public:
    SPITransaction(SPIBusInterface& bus, SPIBusConfig config, GpioPin cs);
    SPITransaction(SPISlave slave);

    void write(uint8_t reg, uint8_t val);
    void write(uint8_t reg, uint8_t* data, size_t size);
    void write(uint8_t* data, size_t size);

    uint8_t read(uint8_t reg);
    void read(uint8_t reg, uint8_t* data, size_t size);
    void read(uint8_t* data, size_t size);

private:
    SPIBusInterface& bus;
    GpioPin cs;
    // No public copy constructor or operator =
    SPITransaction(const SPITransaction& l);
    SPITransaction& operator=(const SPITransaction& l);
};

// Defined here and not in the .cpp to make them inline
inline void SPIBus::write(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        transfer(data + i);
    }
}

inline void SPIBus::read(uint8_t* data, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        read(data + i);
    }
}

inline void SPIBus::select(GpioPin cs)
{
    cs.low();
    if (last_config.cs_setup_time_us > 0)
    {
        delayUs(last_config.cs_setup_time_us);
    }
}

inline void SPIBus::deselect(GpioPin cs)
{
    if (last_config.cs_hold_time_us > 0)
    {
        delayUs(last_config.cs_hold_time_us);
    }
    cs.high();
}

inline void SPIBus::transfer(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while (!(spi->SR | SPI_SR_TXE))
        ;
    // Write the byte in the transmit buffer
    spi->DR = *byte;

    // Wait until byte is transmitted
    while (!(spi->SR | SPI_SR_RXNE))
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}

inline void SPIBus::read(uint8_t* byte)
{
    // Wait until the peripheral is ready to transmit
    while (!(spi->SR | SPI_SR_TXE))
        ;
    // Write 0 in the transmit buffer
    spi->DR = 0;

    // Wait until byte is received
    while (!(spi->SR | SPI_SR_RXNE))
        ;

    // Store the received data in the byte
    *byte = (uint8_t)spi->DR;
}
