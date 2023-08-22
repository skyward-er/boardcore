/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "WIZ5500.h"

#include "interfaces/endianness.h"

using namespace Boardcore;

SPIBusConfig getSpiBusConfig(SPI::ClockDivider clock_divider)
{
    SPIBusConfig bus_config = {};
    bus_config.clockDivider = clock_divider;
    bus_config.mode         = SPI::Mode::MODE_0;
    bus_config.bitOrder     = SPI::Order::MSB_FIRST;
    bus_config.byteOrder    = SPI::Order::MSB_FIRST;

    return bus_config;
}

WizCore::WizCore(SPIBus &bus, miosix::GpioPin cs,
                 SPI::ClockDivider clock_divider)
    : slave(bus, cs, getSpiBusConfig(clock_divider))
{

}

void WizCore::spiRead(uint8_t block, uint16_t address, uint8_t *data, size_t len) {
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write((block & 0b1111) << 3);
    slave.bus.read(data, len);
    slave.bus.deselect(slave.cs);
}

void WizCore::spiWrite(uint8_t block, uint16_t address, const uint8_t *data, size_t len) {
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write((block & 0b1111) << 3 | (1 << 2));
    slave.bus.write(data, len);
    slave.bus.deselect(slave.cs);
}

uint8_t WizCore::spiRead8(uint8_t block, uint16_t address) {
    uint8_t data;
    spiRead(block, address, &data, 1);
    return data;
}

uint16_t WizCore::spiRead16(uint8_t block, uint16_t address) {
    uint16_t data;
    spiRead(block, address, reinterpret_cast<uint8_t*>(&data), 2);
    return fromBigEndian16(data);
}

void WizCore::spiWrite8(uint8_t block, uint16_t address, uint8_t data) {
    spiWrite(block, address, &data, 1);
}

void WizCore::spiWrite16(uint8_t block, uint16_t address, uint16_t data) {
    data = toBigEndian16(data);
    spiWrite(block, address, reinterpret_cast<uint8_t*>(&data), 2);
}