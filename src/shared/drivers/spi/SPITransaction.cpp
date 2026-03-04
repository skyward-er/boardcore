/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

uint8_t SPITransaction::transfer(uint8_t data)
{
    transferImpl(&data, &data, sizeof(data));
    return data;
}

uint16_t SPITransaction::transfer16(uint16_t data)
{
    uint8_t buf[2] = {static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, buf, sizeof(buf));
    return buf[0] << 8 | buf[1];
}

uint32_t SPITransaction::transfer24(uint32_t data)
{
    uint8_t buf[3] = {static_cast<uint8_t>(data >> 16),
                      static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, buf, sizeof(buf));
    return buf[0] << 16 | buf[1] << 8 | buf[2];
}

uint32_t SPITransaction::transfer32(uint32_t data)
{
    uint8_t buf[4] = {
        static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
        static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
    transferImpl(buf, buf, sizeof(buf));
    return buf[0] << 24 | buf[1] << 16 | buf[2] << 8 | buf[3];
}

uint8_t SPITransaction::read()
{
    uint8_t data;
    transferImpl(nullptr, &data, sizeof(data));
    return data;
}

uint16_t SPITransaction::read16()
{
    uint8_t data[2];
    transferImpl(nullptr, data, sizeof(data));
    return data[0] << 8 | data[1];
}

uint32_t SPITransaction::read24()
{
    uint8_t data[3];
    transferImpl(nullptr, data, sizeof(data));
    return data[0] << 16 | data[1] << 8 | data[2];
}

uint32_t SPITransaction::read32()
{
    uint8_t data[4];
    transferImpl(nullptr, data, sizeof(data));
    return data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
}

void SPITransaction::write(uint8_t data)
{
    transferImpl(&data, nullptr, sizeof(data));
}

void SPITransaction::write16(uint16_t data)
{
    // Prepare data in MSB first order
    uint8_t buf[2] = {static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::write24(uint32_t data)
{
    // Prepare data in MSB first order
    uint8_t buf[3] = {static_cast<uint8_t>(data >> 16),
                      static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::write32(uint32_t data)
{
    // Prepare data in MSB first order
    uint8_t buf[4] = {
        static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
        static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

uint8_t SPITransaction::readRegister(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    uint8_t buf[2] = {reg};
    transferImpl(buf, buf, sizeof(buf));

    return buf[1];
}

uint16_t SPITransaction::readRegister16(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    uint8_t buf[3] = {reg};
    transferImpl(buf, buf, sizeof(buf));

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        return buf[2] << 8 | buf[1];
    else
        return buf[1] << 8 | buf[2];
}

uint32_t SPITransaction::readRegister24(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    uint8_t buf[4] = {reg};
    transferImpl(buf, buf, sizeof(buf));

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        return buf[3] << 16 | buf[2] << 8 | buf[1];
    else
        return buf[1] << 16 | buf[2] << 8 | buf[3];
}

uint32_t SPITransaction::readRegister32(uint8_t reg)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    uint8_t buf[5] = {reg};
    transferImpl(buf, buf, sizeof(buf));

    if (slave.config.byteOrder == SPI::Order::LSB_FIRST)
        return buf[4] << 24 | buf[3] << 16 | buf[2] << 8 | buf[1];
    else
        return buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
}

void SPITransaction::writeRegister(uint8_t reg, uint8_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    uint8_t buf[2] = {reg, data};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::writeRegister16(uint8_t reg, uint16_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    // Data needs to be sent MSB first
    uint8_t buf[3] = {reg, static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::writeRegister24(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    // Data needs to be sent MSB first
    uint8_t buf[4] = {reg, static_cast<uint8_t>(data >> 16),
                      static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::writeRegister32(uint8_t reg, uint32_t data)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    // Data needs to be sent MSB first
    uint8_t buf[5] = {
        reg, static_cast<uint8_t>(data >> 24), static_cast<uint8_t>(data >> 16),
        static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data)};
    transferImpl(buf, nullptr, sizeof(buf));
}

void SPITransaction::readRegisters(uint8_t reg, uint8_t* out, size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::NORMAL)
        reg |= 0x80;

    TransactionSelectGuard guard(slave, !externallySelected);
    slave.bus.transfer(&reg, nullptr, sizeof(reg));
    slave.bus.transfer(nullptr, out, size);
}

void SPITransaction::writeRegisters(uint8_t reg, const uint8_t* data,
                                    size_t size)
{
    if (slave.config.writeBit == SPI::WriteBit::INVERTED)
        reg |= 0x80;

    TransactionSelectGuard guard(slave, !externallySelected);
    slave.bus.transfer(&reg, nullptr, sizeof(reg));
    slave.bus.transfer(data, nullptr, size);
}

}  // namespace Boardcore
