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

namespace Boardcore
{

SPITransaction::SPITransaction(SPISlave slave, WriteBit writeBit)
    : SPITransaction(slave.bus, slave.cs, slave.config, writeBit)
{
}

SPITransaction::SPITransaction(SPIBusInterface& bus, GpioType cs,
                               SPIBusConfig config, WriteBit writeBit)
    : bus(bus), writeBit(writeBit), cs(cs)
{
    bus.configure(config);
}

SPIBusInterface& SPITransaction::getBus() { return bus; }

// Read, write and transfer operations in master mode

uint8_t SPITransaction::read()
{
    bus.select(cs);
    uint8_t data = bus.read();
    bus.deselect(cs);
    return data;
}

uint16_t SPITransaction::read16()
{
    bus.select(cs);
    uint16_t data = bus.read16();
    bus.deselect(cs);
    return data;
}

void SPITransaction::read(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.read(data, size);
    bus.deselect(cs);
}

void SPITransaction::read(uint16_t* data, size_t size)
{
    bus.select(cs);
    bus.read(data, size);
    bus.deselect(cs);
}

void SPITransaction::write(uint8_t data)
{
    bus.select(cs);
    bus.write(data);
    bus.deselect(cs);
}

void SPITransaction::write(uint16_t data)
{
    bus.select(cs);
    bus.write(data);
    bus.deselect(cs);
}

void SPITransaction::write(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.write(data, size);
    bus.deselect(cs);
}

void SPITransaction::write(uint16_t* data, size_t size)
{
    bus.select(cs);
    bus.write(data, size);
    bus.deselect(cs);
}

uint8_t SPITransaction::transfer(uint8_t data)
{
    bus.select(cs);
    data = bus.transfer(data);
    bus.deselect(cs);
    return data;
}

uint16_t SPITransaction::transfer(uint16_t data)
{
    bus.select(cs);
    data = bus.transfer(data);
    bus.deselect(cs);
    return data;
}

void SPITransaction::transfer(uint8_t* data, size_t size)
{
    bus.select(cs);
    bus.transfer(data, size);
    bus.deselect(cs);
}

void SPITransaction::transfer(uint16_t* data, size_t size)
{
    bus.select(cs);
    bus.transfer(data, size);
    bus.deselect(cs);
}

// Read, write and transfer operations with registers

uint8_t SPITransaction::readRegister(uint8_t reg)
{
    if (writeBit == WriteBit::NORMAL)
        reg |= 0x80;

    bus.select(cs);
    bus.write(reg);
    uint8_t data = bus.read();
    bus.deselect(cs);
    return data;
}

void SPITransaction::readRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (writeBit == WriteBit::NORMAL)
        reg |= 0x80;

    bus.select(cs);
    bus.write(reg);
    bus.read(data, size);
    bus.deselect(cs);
}

void SPITransaction::writeRegister(uint8_t reg, uint8_t data)
{
    if (writeBit == WriteBit::INVERTED)
        reg |= 0x80;

    bus.select(cs);
    bus.write(reg);
    bus.write(data);
    bus.deselect(cs);
}

void SPITransaction::writeRegisters(uint8_t reg, uint8_t* data, size_t size)
{
    if (writeBit == WriteBit::INVERTED)
        reg |= 0x80;

    bus.select(cs);
    bus.write(reg);
    bus.write(data, size);
    bus.deselect(cs);
}

}  // namespace Boardcore
