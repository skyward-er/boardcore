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

#include "WIZ5500Defs.h"
#include "interfaces/endianness.h"

using namespace Boardcore;
using namespace miosix;

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

bool WizCore::checkVersion()
{
    return spiRead8(0, Wiz::Common::REG_VERSIONR) == Wiz::VERSION;
}

void WizCore::softReset()
{
    // Perform a software reset
    spiWrite8(0, Wiz::Common::REG_MR, 1 << 7);
    Thread::sleep(2);
}

void WizCore::setGatewayIp(WizIp ip)
{
    spiWriteIp(0, Wiz::Common::REG_GAR, ip);
}

void WizCore::setSubnetMask(WizIp mask)
{
    spiWriteIp(0, Wiz::Common::REG_SUBR, mask);
}

void WizCore::setSourceMac(WizMac mac)
{
    spiWriteMac(0, Wiz::Common::REG_SHAR, mac);
}

void WizCore::setSourceIp(WizIp ip)
{
    spiWriteIp(0, Wiz::Common::REG_SIPR, ip);
}

bool WizCore::connectTcp(int sock_n, uint16_t source_port, WizIp destination_ip,
                         uint16_t destination_port, int timeout)
{
    // First setup the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_MR, 0b0001);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_PORT,
               source_port);
    spiWriteIp(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DIPR,
               destination_ip);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_DPORT,
               destination_port);

    // Then open the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_OPEN);
    Thread::sleep(200);  // TODO: Actually change this to an interrupt wait

    // Finally connect the socket
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_CONNECT);
    Thread::sleep(200);

    // Now check that it is actually established
    return spiRead8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_SR) ==
           Wiz::Socket::STAT_ESTABLISHED;
}

void WizCore::send(int sock_n, const uint8_t *data, size_t len,
                   int timeout)
{
    // First get the start write address
    uint16_t start_addr =
        spiRead16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_TX_WR);

    // Fill the TX buffer and update the counter
    spiWrite(Wiz::getSocketTxBlock(sock_n), start_addr, data, len);
    spiWrite16(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_TX_WR,
               start_addr + len);

    // Finally tell the device to send the data
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_SEND);
}

void WizCore::close(int sock_n, int timeout)
{
    spiWrite8(Wiz::getSocketRegBlock(sock_n), Wiz::Socket::REG_CR,
              Wiz::Socket::CMD_DISCON);
    Thread::sleep(200);
}

void WizCore::spiRead(uint8_t block, uint16_t address, uint8_t *data,
                      size_t len)
{
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write(Wiz::buildControlWord(block, false));
    slave.bus.read(data, len);
    slave.bus.deselect(slave.cs);
}

void WizCore::spiWrite(uint8_t block, uint16_t address, const uint8_t *data,
                       size_t len)
{
    // Do a manual SPI transaction
    slave.bus.configure(slave.config);

    slave.bus.select(slave.cs);
    slave.bus.write16(address);
    slave.bus.write(Wiz::buildControlWord(block, true));
    slave.bus.write(data, len);
    slave.bus.deselect(slave.cs);
}

uint8_t WizCore::spiRead8(uint8_t block, uint16_t address)
{
    uint8_t data;
    spiRead(block, address, &data, 1);
    return data;
}

uint16_t WizCore::spiRead16(uint8_t block, uint16_t address)
{
    uint16_t data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data),
            sizeof(uint16_t));
    return fromBigEndian16(data);
}

WizIp WizCore::spiReadIp(uint8_t block, uint16_t address)
{
    WizIp data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizIp));
    return data;
}

WizMac WizCore::spiReadMac(uint8_t block, uint16_t address)
{
    WizMac data;
    spiRead(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizMac));
    return data;
}

void WizCore::spiWrite8(uint8_t block, uint16_t address, uint8_t data)
{
    spiWrite(block, address, &data, 1);
}

void WizCore::spiWrite16(uint8_t block, uint16_t address, uint16_t data)
{
    data = toBigEndian16(data);
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data),
             sizeof(uint16_t));
}

void WizCore::spiWriteIp(uint8_t block, uint16_t address, WizIp data)
{
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data), sizeof(WizIp));
}

void WizCore::spiWriteMac(uint8_t block, uint16_t address, WizMac data)
{
    spiWrite(block, address, reinterpret_cast<uint8_t *>(&data),
             sizeof(WizMac));
}