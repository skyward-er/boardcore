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
using intn = Gpio<GPIOF_BASE, 6>;

SPIBus bus(SPI4);

#define INTN_IRQ EXTI6_IRQHandlerImpl

#else
#error "Target not supported"
#endif

WizCore *wiz = nullptr;

#ifdef INTN_IRQ
void __attribute__((used)) INTN_IRQ()
{
    if (wiz)
        wiz->handleINTn();
}
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
    intn::mode(Mode::INPUT);
}

int main() {
    setupBoard();

    wiz = new WizCore(bus, cs::getPin(), intn::getPin(), SPI::ClockDivider::DIV_64);

    // Start the driver
    if(!wiz->start()) {
        printf("[wiz5500] Wiz failed to start!\n");
        while(1);
    }

    wiz->setGatewayIp({192, 168, 1, 1});
    wiz->setSubnetMask({255, 255, 225, 0});
    wiz->setSourceMac({0x00, 0x08, 0xdc, 0x01, 0x02, 0x03});
    wiz->setSourceIp({192, 168, 1, 69});

    printf("[wiz5500] Connecting...\n");
    bool opened = wiz->listenTcp(0, 8080);

    if(opened) {
        printf("[wiz5500] Sending data...\n");
        // Now send tha famous words
        const char *out_msg = "Suca palle (DIO0)\n";
        wiz->send(0, reinterpret_cast<const uint8_t*>(out_msg), strlen(out_msg));

        // Try to read something back
        char in_msg[1024];
        ssize_t len = wiz->recv(0, reinterpret_cast<uint8_t*>(in_msg), sizeof(in_msg));

        if(len != -1) {
            in_msg[len] = '\0';
            printf("[wiz5500] Received: %s\n", in_msg);
        } else {
            printf("[wiz5500] Failed to receive\n");
        }

        Thread::sleep(1000);
        printf("[wiz5500] Closing...\n");
        wiz->close(0);   
    } else {
        printf("[wiz5500] Failed to send data!\n");
    }

    while(1);
    return 0;
}