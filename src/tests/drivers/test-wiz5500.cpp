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

#include <drivers/WIZ5500/WIZ5500.h>
#include <drivers/WIZ5500/WIZ5500Defs.h>

#include <cstring>
#include <iostream>
#include <thread>

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_STM32F4DISCOVERY

using cs   = Gpio<GPIOC_BASE, 13>;
using sck  = Gpio<GPIOE_BASE, 2>;
using miso = Gpio<GPIOE_BASE, 5>;
using mosi = Gpio<GPIOE_BASE, 6>;
using intn = Gpio<GPIOF_BASE, 6>;

SPIBus bus(SPI4);

#define INTN_IRQ EXTI6_IRQHandlerImpl

void setupBoard()
{
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

#elif defined _BOARD_STM32F767ZI_GEMINI_GS
#include "interfaces-impl/hwmapping.h"

using cs   = ethernet::cs;
using sck  = ethernet::spi::sck;
using miso = ethernet::spi::miso;
using mosi = ethernet::spi::mosi;
using intn = ethernet::intr;

SPIBus bus(MIOSIX_ETHERNET_SPI);

#define INTN_IRQ MIOSIX_ETHERNET_IRQ

void setupBoard() {}

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

void socket0SendLoop()
{
    int i = 0;
    while (true)
    {
        char msg[1024];
        size_t len = sprintf(msg, "Suca palle (DIO0) (0 %d)\n", i);

        printf("[wiz5500] Sending though socket 0...\n");
        wiz->send(0, reinterpret_cast<uint8_t *>(msg), len);
        Thread::sleep(2000);
        i += 1;
    }
}

void socket0RecvLoop()
{
    while (true)
    {
        char msg[1024];
        ssize_t len =
            wiz->recv(0, reinterpret_cast<uint8_t *>(msg), sizeof(msg));

        if (len != -1)
        {
            msg[len] = '\0';
            printf("[wiz5500] Received \"%s\" from socket 0\n", msg);
        }
        else
        {
            Thread::sleep(1000);
        }
    }
}

void socket0Start()
{
    WizIp dst_ip;
    uint16_t dst_port;

    printf("[wiz5500] Opening socket 0...\n");
    bool opened = wiz->listenTcp(0, 8080, dst_ip, dst_port);
    std::cout << dst_ip << " " << dst_port << std::endl;
    if (opened)
    {
        std::thread t(socket0SendLoop);
        socket0RecvLoop();
    }
    else
    {
        printf("[wiz5500] Failed to open socket 0!\n");
    }
}

void socket1SendLoop()
{
    int i = 0;
    while (true)
    {
        char msg[1024];
        size_t len = sprintf(msg, "Suca palle (DIO0) (1 %d)\n", i);

        printf("[wiz5500] Sending though socket 1...\n");
        wiz->send(1, reinterpret_cast<uint8_t *>(msg), len);
        Thread::sleep(1333);
        i += 1;
    }
}

void socket1RecvLoop()
{
    while (true)
    {
        char msg[1024];
        ssize_t len =
            wiz->recv(1, reinterpret_cast<uint8_t *>(msg), sizeof(msg));

        if (len != -1)
        {
            msg[len] = '\0';
            printf("[wiz5500] Received \"%s\" from socket 1\n", msg);
        }
        else
        {
            Thread::sleep(1000);
        }
    }
}

void socket1Start()
{
    printf("[wiz5500] Opening socket 1...\n");
    bool opened = wiz->connectTcp(1, 8081, {192, 168, 1, 12}, 8080);
    if (opened)
    {
        std::thread t(socket1SendLoop);
        socket1RecvLoop();
    }
    else
    {
        printf("[wiz5500] Failed to open socket 1!\n");
    }
}

void socket2SendLoop()
{
    int i = 0;
    while (true)
    {
        char msg[1024];
        size_t len = sprintf(msg, "Suca palle (DIO0) (2 %d)\n", i);

        printf("[wiz5500] Sending though socket 2...\n");
        wiz->send(2, reinterpret_cast<uint8_t *>(msg), len);
        Thread::sleep(1666);
        i += 1;
    }
}

void socket2RecvLoop()
{
    while (true)
    {
        char msg[1024];
        WizIp dst_ip;
        uint16_t dst_port;
        ssize_t len = wiz->recvfrom(2, reinterpret_cast<uint8_t *>(msg),
                                    sizeof(msg), dst_ip, dst_port);

        std::cout << dst_ip << " " << dst_port << std::endl;

        if (len != -1)
        {
            msg[len] = '\0';
            printf("[wiz5500] Received \"%s\" from socket 2\n", msg);
        }
        else
        {
            Thread::sleep(1000);
        }
    }
}

void socket2Start()
{
    printf("[wiz5500] Opening socket 2...\n");
    bool opened = wiz->openUdp(2, 8081, {192, 168, 1, 12}, 8080);
    if (opened)
    {
        std::thread t(socket2SendLoop);
        socket2RecvLoop();
    }
    else
    {
        printf("[wiz5500] Failed to open socket 2!\n");
    }
}

int main()
{
    setupBoard();

    wiz = new WizCore(bus, cs::getPin(), intn::getPin(),
                      SPI::ClockDivider::DIV_64);

    // Start the driver
    if (!wiz->start())
    {
        printf("[wiz5500] Wiz failed to start!\n");
        while (1)
            ;
    }

    wiz->setGatewayIp({192, 168, 1, 1});
    wiz->setSubnetMask({255, 255, 225, 0});
    wiz->setSourceMac({0x00, 0x08, 0xdc, 0x01, 0x02, 0x03});
    wiz->setSourceIp({192, 168, 1, 69});

    std::thread t1(socket0Start);
    std::thread t2(socket1Start);
    socket2Start();

    while (1)
        ;
    return 0;
}