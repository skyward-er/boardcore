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

#include "SPIDriver.h"

SPIBus::SPIBus(SPI_TypeDef* spi) : spi(spi) {}

void SPIBus::configure(SPIBusConfig new_config)
{
    config = new_config;

    // Clean CR1
    spi->CR1 = 0;

    // Configure clock division (BR bits)
    spi->CR1 |= (static_cast<uint16_t>(config.br) & 0x0003) << 3;
    // Configure CPOL & CPHA bits
    spi->CR1 |= (uint16_t)config.cpol << 1 | (uint16_t)config.cpha;

    // Configure LSBFIRST bit
    spi->CR1 |= (uint16_t)config.lsb_first << 7;

    spi->CR1 |= SPI_CR1_SSI | SPI_CR1_SSM  // Use software chip-select
                | SPI_CR1_MSTR             // Master mode
                | SPI_CR1_SPE;             // Enable SPI
}

SPITransaction::SPITransaction(SPIBusInterface& bus, GpioPin cs,
                               SPIBusConfig config)
    : bus(bus), cs(cs)
{
    bus.configure(config);
}

SPITransaction::SPITransaction(SPISlave slave)
    : SPITransaction(slave.bus, slave.cs, slave.config)
{
}

void SPITransaction::write(uint8_t cmd)
{
    bus.select(cs);
    bus.write(&cmd, 1);
    bus.deselect(cs);
}

void SPITransaction::write(uint8_t reg, uint8_t val)
{
    bus.select(cs);
    bus.write(&reg, 1);
    bus.write(&val, 1);
    bus.deselect(cs);
}

void SPITransaction::write(uint8_t reg, uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.write(&reg, 1);
    bus.write(data, size);
    bus.deselect(cs);
}

void SPITransaction::write(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.write(data, size);
    bus.deselect(cs);
}

void SPITransaction::transfer(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.transfer(data, size);
    bus.deselect(cs);
}

uint8_t SPITransaction::read(uint8_t reg, bool set_read_bit)
{
    if (set_read_bit)
        reg = reg | 0x80;

    uint8_t out;
    bus.select(cs);
    bus.write(&reg, 1);
    bus.read(&out, 1);
    bus.deselect(cs);
    return out;
}

void SPITransaction::read(uint8_t reg, uint8_t* data, size_t size,
                          bool set_read_bit)
{
    if (set_read_bit)
        reg = reg | 0x80;

    bus.select(cs);
    bus.write(&reg, 1);
    bus.read(data, size);
    bus.deselect(cs);
}

void SPITransaction::read(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.read(data, size);
    bus.deselect(cs);
}