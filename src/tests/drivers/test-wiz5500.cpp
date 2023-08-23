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
    if(!wiz.checkVersion()) {
        printf("[wiz5500] Version check failed!\n");
        while(1);
    }

    // Do a quick software reset
    wiz.softReset();

    wiz.setGatewayIp({192, 168, 1, 1});
    wiz.setSubnetMask({255, 255, 225, 0});
    wiz.setSourceMac({0x00, 0x08, 0xdc, 0x01, 0x02, 0x03});
    wiz.setSourceIp({192, 168, 1, 69});

    printf("[wiz5500] Connecting...\n");
    wiz.connectTcp(0, 13456, {192, 168, 1, 12}, 8080);

    printf("[wiz5500] Sending data...\n");
    // Now send tha famous words
    const char *msg = "Suca palle (DIO0)";
    wiz.send(0, reinterpret_cast<const uint8_t*>(msg), strlen(msg));

    Thread::sleep(1000);
    printf("[wiz5500] Closing...\n");
    wiz.close(0);

    while(1);
    return 0;
}