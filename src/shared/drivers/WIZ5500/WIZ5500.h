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

#pragma once

#include <drivers/spi/SPIDriver.h>
#include <ActiveObject.h>

namespace Boardcore
{

struct WizIp
{
    uint8_t a, b, c, d;
};

struct WizMac
{
    uint8_t a, b, c, d, e, f;
};

class WizCore
{
public:
    WizCore(SPIBus& bus, miosix::GpioPin cs, SPI::ClockDivider clock_divider);

    void spiRead(uint8_t block, uint16_t address, uint8_t* data, size_t len);
    void spiWrite(uint8_t block, uint16_t address, const uint8_t* data,
                  size_t len);

    uint8_t spiRead8(uint8_t block, uint16_t address);
    uint16_t spiRead16(uint8_t block, uint16_t address);
    WizIp spiReadIp(uint8_t block, uint16_t address);
    WizMac spiReadMac(uint8_t block, uint16_t address);

    void spiWrite8(uint8_t block, uint16_t address, uint8_t data);
    void spiWrite16(uint8_t block, uint16_t address, uint16_t data);
    void spiWriteIp(uint8_t block, uint16_t address, WizIp data);
    void spiWriteMac(uint8_t block, uint16_t address, WizMac data);

private:
    SPISlave slave;
};

}  // namespace Boardcore

namespace std
{
inline ostream& operator<<(ostream& os, const Boardcore::WizIp& ip)
{
    auto old_flags = os.flags(os.dec);
    os << (int)ip.a << "." << (int)ip.b << "." << (int)ip.c << "." << (int)ip.d;
    os.flags(old_flags);
    return os;
}

inline ostream& operator<<(ostream& os, const Boardcore::WizMac& mac)
{
    auto old_flags = os.flags(os.hex);
    os << (int)mac.a << ":" << (int)mac.b << ":" << (int)mac.c
       << ":" << (int)mac.d << ":" << (int)mac.e << ":" << (int)mac.f;
    os.flags(old_flags);
    return os;
}
}  // namespace std