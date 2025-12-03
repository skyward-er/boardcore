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

#include <interfaces/endianness.h>

namespace Boardcore
{

template <typename AddressSize>
SPITransaction<AddressSize>::SPITransaction(const SPISlave<AddressSize>& slave)
    : slave(slave)
{
    slave.bus.configure(slave.config);
}

template <typename AddressSize>
SPIBusInterface& SPITransaction<AddressSize>::getBus()
{
    return slave.bus;
}

// Read, write and transfer operations in master mode

template <typename AddressSize>
template <typename DataSize>
DataSize SPITransaction<AddressSize>::read()
{
    slave.bus.select(slave.cs);
    uint8_t data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    return data;
}

template <typename AddressSize>
template <typename DataSize>
void SPITransaction<AddressSize>::read(DataSize* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename DataSize>
void SPITransaction<AddressSize>::write(DataSize data)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename DataSize>
void SPITransaction<AddressSize>::write(DataSize* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename DataSize>
DataSize SPITransaction<AddressSize>::transfer(DataSize data)
{
    slave.bus.select(slave.cs);
    data = slave.bus.transfer(data);
    slave.bus.deselect(slave.cs);

    return data;
}

template <typename AddressSize>
template <typename DataSize>
void SPITransaction<AddressSize>::transfer(DataSize* data, size_t size)
{
    slave.bus.select(slave.cs);
    slave.bus.transfer(data, size);
    slave.bus.deselect(slave.cs);
}

// Read, write and transfer operations with registers
template <typename AddressSize>
template <typename RegisterSize>
RegisterSize SPITransaction<AddressSize>::readRegister(AddressSize reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg = (reg & ~slave.readWriteMask) | slave.readWriteMask;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    RegisterSize data = slave.bus.read();
    slave.bus.deselect(slave.cs);
    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    return data;
}

template <typename AddressSize>
template <typename RegisterSize>
void SPITransaction<AddressSize>::readRegisters(AddressSize reg,
                                                RegisterSize* data, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg = (reg & ~slave.readWriteMask) | slave.readWriteMask;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename RegisterSize>
void SPITransaction<AddressSize>::writeRegister(AddressSize reg,
                                                RegisterSize data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg = (reg & ~slave.readWriteMask) | slave.readWriteMask;

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename RegisterSize>
void SPITransaction<AddressSize>::writeRegisters(AddressSize reg,
                                                 RegisterSize* data,
                                                 size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg = (reg & ~slave.readWriteMask) | slave.readWriteMask;

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
template <typename DataSize>
DataSize SPITransaction<AddressSize>::swapBytes(DataSize data)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "Data should be 8, 16 or 32 bit (Available types: uint8_t, "
                  "uint16_t, uint32_t)");
}

template <typename AddressSize>
uint8_t SPITransaction<AddressSize>::swapBytes(uint8_t data)
{
    return data;
}

template <typename AddressSize>
uint16_t SPITransaction<AddressSize>::swapBytes(uint16_t data)
{
    return swapBytes16(data);
}

template <typename AddressSize>
uint32_t SPITransaction<AddressSize>::swapBytes(uint32_t data)
{
    return swapBytes32(data);
}

}  // namespace Boardcore
