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

#include <iostream>
#include <cstring>

#include <drivers/WIZ5500/WIZ5500.h>
#include <drivers/WIZ5500/WIZ5500Defs.h>

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_STM32F4DISCOVERY

using cs = Gpio<GPIOC_BASE, 13>;
using sck = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;

SPIBus bus(SPI4);

#else
#error "Target not supported"
#endif

void setupBoard() {
    sck::mode(Mode::ALTERNATE);
    sck::alternateFunction(5);
    miso::mode(Mode::ALTERNATE);
    miso::alternateFunction(5);
    mosi::mode(Mode::ALTERNATE);
    mosi::alternateFunction(5);
    cs::mode(Mode::OUTPUT);
    cs::high();
}

int main() {
    setupBoard();

    WizCore wiz = WizCore(bus, cs::getPin(), SPI::ClockDivider::DIV_64);
    uint8_t version = wiz.spiRead8(0x00, Wiz::Common::REG_VERSIONR);
    printf("Version: %d\n", version);

    // Quickly reset the device
    wiz.spiWrite8(0, Wiz::Common::REG_MR, 0x00 | 1 << 7);
    Thread::sleep(100);

    uint16_t rtr = wiz.spiRead16(0x00, Wiz::Common::REG_RTR);
    printf("RTR: %x\n", rtr);

    std::cout << "Mac: " << wiz.spiReadMac(0, Wiz::Common::REG_GAR) << std::endl;

    wiz.spiWrite8(0, Wiz::Common::REG_MR, 0x00);
    wiz.spiWriteIp(0, Wiz::Common::REG_GAR, {192, 168, 1, 1});
    wiz.spiWriteIp(0, Wiz::Common::REG_SUBR, {255, 255, 225, 0});
    wiz.spiWriteMac(0, Wiz::Common::REG_SHAR, {0x00, 0x08, 0xdc, 0x01, 0x02, 0x03});
    wiz.spiWriteIp(0, Wiz::Common::REG_SIPR, {192, 168, 1, 69});

    std::cout << "Gateway: " << wiz.spiReadIp(0, Wiz::Common::REG_GAR) << std::endl;
    std::cout << "Subnet: " << wiz.spiReadIp(0, Wiz::Common::REG_SUBR) << std::endl;
    std::cout << "Mac: " << wiz.spiReadMac(0, Wiz::Common::REG_SHAR) << std::endl;
    std::cout << "Ip: " << wiz.spiReadIp(0, Wiz::Common::REG_SIPR) << std::endl;

    wiz.spiWrite8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_MR, 0b0001);

    std::cout << "Status: " << (int)wiz.spiRead8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_SR) << std::endl;

    wiz.spiWrite16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_PORT, 13456);
    wiz.spiWriteIp(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_DIPR, {192, 168, 1, 12});
    wiz.spiWrite16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_DPORT, 8080);

    // First open the socket
    wiz.spiWrite8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_CR, Wiz::Socket::CMD_OPEN);
    Thread::sleep(200);
    std::cout << "Status: " << (int)wiz.spiRead8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_SR) << std::endl;
    wiz.spiWrite8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_CR, Wiz::Socket::CMD_CONNECT);
    Thread::sleep(200);
    std::cout << "Status: " << (int)wiz.spiRead8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_SR) << std::endl;
    
    // Now send tha famous words
    uint16_t start_addr = wiz.spiRead16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_TX_WR);
    std::cout << "TX_WR: " << wiz.spiRead16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_TX_WR) << std::endl;

    const char *msg = "Suca palle (DIO0)";
    wiz.spiWrite(Wiz::getSocketTxBlock(0), start_addr, reinterpret_cast<const uint8_t*>(msg), strlen(msg));

    wiz.spiWrite16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_TX_WR, start_addr + strlen(msg));
    std::cout << "TX_WR: " << wiz.spiRead16(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_TX_WR) << std::endl;

    // Ok now tell the device to send the data
    wiz.spiWrite8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_CR, Wiz::Socket::CMD_SEND);
    Thread::sleep(200);
    std::cout << "Status: " << (int)wiz.spiRead8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_SR) << std::endl;

    wiz.spiWrite8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_CR, Wiz::Socket::CMD_DISCON);
    Thread::sleep(200);
    std::cout << "Status: " << (int)wiz.spiRead8(Wiz::getSocketRegBlock(0), Wiz::Socket::REG_SR) << std::endl;

    return 0;
}