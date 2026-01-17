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

SPITransaction::SPITransaction(const SPISlave& slave) : slave(slave)
{
    slave.bus.configure(slave.config);
}

SPIBusInterface& SPITransaction::getBus() { return slave.bus; }

// Read, write and transfer operations in master mode

template <typename DataSize>
DataSize SPITransaction::read()
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");

    slave.bus.select(slave.cs);
    DataSize data = slave.bus.read();
    slave.bus.deselect(slave.cs);
    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    return data;
}

template <typename DataSize>
void SPITransaction::read(DataSize* data, size_t size)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");

    slave.bus.select(slave.cs);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);
    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size = size - 1; size >= 0; size--)
            data[size] = swapBytes(data[size]);
}

template <typename DataSize>
void SPITransaction::write(DataSize data)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");
    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);
    slave.bus.select(slave.cs);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

template <typename DataSize>
void SPITransaction::write(DataSize* data, size_t size)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");
    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size_t i = 0; i < size; i++)
            data[size] = swapBytes(data[size]);

    slave.bus.select(slave.cs);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename DataSize>
DataSize SPITransaction::transfer(DataSize data)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    slave.bus.select(slave.cs);
    data = slave.bus.transfer(data);
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    return data;
}

template <typename DataSize>
void SPITransaction::transfer(DataSize* data, size_t size)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size_t i = 0; i < size; i++)
            data[size] = swapBytes(data[size]);

    slave.bus.select(slave.cs);
    slave.bus.transfer(data, size);
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size_t i = 0; i < size; i++)
            data[size] = swapBytes(data[size]);
}

// Read, write and transfer operations with registers
template <typename AddressSize, typename RegisterSize>
RegisterSize SPITransaction::readRegister(AddressSize reg)
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t.");
    static_assert(std::is_same<RegisterSize, uint8_t>::value ||
                      std::is_same<RegisterSize, uint16_t>::value ||
                      std::is_same<RegisterSize, uint32_t>::value,
                  "RegisterSize can be one of uint8_t, uint16_t, uint32_t.");

    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg = applyReadWriteMask(reg);

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    RegisterSize data = slave.bus.read();
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    return data;
}

template <typename AddressSize, typename RegisterSize>
void SPITransaction::readRegisters(AddressSize reg, RegisterSize* data,
                                   size_t size)
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t.");
    static_assert(std::is_same<RegisterSize, uint8_t>::value ||
                      std::is_same<RegisterSize, uint16_t>::value ||
                      std::is_same<RegisterSize, uint32_t>::value,
                  "RegisterSize can be one of uint8_t, uint16_t, uint32_t.");

    AddressSize readWriteMask =
        0b1 << ((8 * sizeof(AddressSize)) -
                static_cast<AddressSize>(slave.config.writeBitPosition));

    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg = applyReadWriteMask(reg);

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.read(data, size);
    slave.bus.deselect(slave.cs);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size = size - 1; size >= 0; size--)
            data[size] = swapBytes(data[size]);
}

template <typename AddressSize, typename RegisterSize>
void SPITransaction::writeRegister(AddressSize reg, RegisterSize data)
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t.");
    static_assert(std::is_same<RegisterSize, uint8_t>::value ||
                      std::is_same<RegisterSize, uint16_t>::value ||
                      std::is_same<RegisterSize, uint32_t>::value,
                  "RegisterSize can be one of uint8_t, uint16_t, uint32_t.");

    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg = applyReadWriteMask(reg);

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        data = swapBytes(data);

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize, typename RegisterSize>
void SPITransaction::writeRegisters(AddressSize reg, RegisterSize* data,
                                    size_t size)
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t.");
    static_assert(std::is_same<RegisterSize, uint8_t>::value ||
                      std::is_same<RegisterSize, uint16_t>::value ||
                      std::is_same<RegisterSize, uint32_t>::value,
                  "RegisterSize can be one of uint8_t, uint16_t, uint32_t.");

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        for (size_t i = 0; i < size; i++)
            data[i] = swapBytes(data[i]);

    slave.bus.select(slave.cs);
    slave.bus.write(reg);
    slave.bus.write(data, size);
    slave.bus.deselect(slave.cs);
}

template <typename AddressSize>
AddressSize SPITransaction::applyReadWriteMask(AddressSize reg)
{
    static_assert(std::is_same<AddressSize, uint8_t>::value ||
                      std::is_same<AddressSize, uint16_t>::value ||
                      std::is_same<AddressSize, uint32_t>::value,
                  "AddressSize can be one of uint8_t, uint16_t, uint32_t.");
    AddressSize readWriteMask =
        0b1 << ((8 * sizeof(AddressSize)) -
                static_cast<AddressSize>(slave.config.writeBitPosition));
    return (reg & ~slave.config.readWriteMask) | slave.config.readWriteMask;
}

template <typename DataSize>
DataSize SPITransaction::swapBytes(DataSize data)
{
    static_assert(std::is_same<DataSize, uint8_t>::value ||
                      std::is_same<DataSize, uint16_t>::value ||
                      std::is_same<DataSize, uint32_t>::value,
                  "DataSize can be one of uint8_t, uint16_t, uint32_t.");
}

uint8_t SPITransaction::swapBytes(uint8_t data) { return data; }

uint16_t SPITransaction::swapBytes(uint16_t data) { return swapBytes16(data); }

uint32_t SPITransaction::swapBytes(uint32_t data) { return swapBytes32(data); }

}  // namespace Boardcore
