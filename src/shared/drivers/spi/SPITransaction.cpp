/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Davide Mor
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

#include "SPITransaction.h"
#include <utils/Debug.h>
#include <interfaces/endianness.h>

namespace Boardcore
{

SPITransaction::SPITransaction(const SPISlave& slave) : slave(slave)
{
    slave.bus.configure(slave.config);
}

SPIBusInterface& SPITransaction::getBus() { return slave.bus; }

// Read, write and transfer operations in master mode

uint8_t SPITransaction::read()
{
    slave.bus.select(slave.cs);
    uint8_t data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::read16()
{
    slave.bus.select(slave.cs);
    uint16_t data = slave.bus.read16();
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::read24()
{
    slave.bus.select(slave.cs);
    uint32_t data = slave.bus.read24();
    slave.bus.deselect(slave.cs);
    return data;
}

uint32_t SPITransaction::read32()
{
    slave.bus.select(slave.cs);
    uint32_t data = slave.bus.read32();
    slave.bus.deselect(slave.cs);
    return data;
}

void SPITransaction::read(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::read16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.read16(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write(uint8_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write16(uint16_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write16(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write24(uint32_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write24(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write32(uint32_t data)
{
    slave.bus.select(slave.cs);
    slave.bus.write32(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::write16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.write16(data, size);
    slave.bus.deselect(slave.cs);
}

uint8_t SPITransaction::transfer(uint8_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::transfer16(uint16_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer16(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::transfer24(uint32_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer24(data);
    slave.bus.deselect(slave.cs);

    return data;
}

uint32_t SPITransaction::transfer32(uint32_t data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer32(data);
    slave.bus.deselect(slave.cs);

    return data;
}

void SPITransaction::transfer(uint8_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.transfer(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::transfer16(uint16_t* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.transfer16(data, size);
    slave.bus.deselect(slave.cs);
}

// Read, write and transfer operations with registers

uint8_t SPITransaction::readRegister(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint8_t data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    return data;
}

uint16_t SPITransaction::readRegister16(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint16_t data = slave.bus.read16();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes16(data);

    return data;
}

uint32_t SPITransaction::readRegister24(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint32_t data = slave.bus.read24();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes32(data) >> 8;

    return data;
}

uint32_t SPITransaction::readRegister32(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    uint32_t data = slave.bus.read32();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes32(data) >> 8;

    return data;
}

void SPITransaction::readRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister(uint8_t reg, uint8_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    TRACE("SPI: writeRegister: reg=%02X, data=%02X\n", reg, data);
    TRACE("selecting slave %d\n", slave.cs);
    slave.bus.select(slave.cs);
    TRACE("writing reg\n");
    slave.bus.write(reg);
    TRACE("writing data\n");
    slave.bus.write(data);
    TRACE("deselecting slave %d\n", slave.cs);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister16(uint8_t reg, uint16_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write16(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister24(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write24(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegister32(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write32(data);
    slave.bus.deselect(slave.cs);
}

void SPITransaction::writeRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

}  // namespace Boardcore
