/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <drivers/interrupt/external_interrupts.h>
#include <filesystem/console/console_device.h>
#include <radio/SX1278/SX1278.h>

#include <thread>

#include "test-sx1278-core.h"

using namespace Boardcore;
using namespace miosix;

SPIBus bus(SPI3);

GpioPin sck(GPIOC_BASE, 10);
GpioPin miso(GPIOC_BASE, 11);
GpioPin mosi(GPIOC_BASE, 12);
GpioPin cs(GPIOA_BASE, 1);
GpioPin dio(GPIOC_BASE, 15);

SX1278* sx1278 = nullptr;

void __attribute__((used)) EXTI15_IRQHandlerImpl()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

/// Initialize stm32f407g board.
void initBoard()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI3
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        RCC_SYNC();

        // Setup SPI pins
        sck.mode(miosix::Mode::ALTERNATE);
        sck.alternateFunction(6);
        miso.mode(miosix::Mode::ALTERNATE);
        miso.alternateFunction(6);
        mosi.mode(miosix::Mode::ALTERNATE);
        mosi.alternateFunction(6);

        cs.mode(miosix::Mode::OUTPUT);
        dio.mode(miosix::Mode::INPUT);
    }

    cs.high();
    enableExternalInterrupt(dio.getPort(), dio.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

void recvLoop()
{
    uint8_t msg[256];
    while (1)
    {
        int len = sx1278->receive(msg, sizeof(msg));
        if (len > 0)
        {
            auto serial = miosix::DefaultConsole::instance().get();
            serial->writeBlock(msg, len, 0);
        }
    }
}

void sendLoop()
{
    uint8_t msg[63];
    while (1)
    {
        auto serial = miosix::DefaultConsole::instance().get();
        int len     = serial->readBlock(msg, sizeof(msg), 0);
        if (len > 0)
        {
            sx1278->send(msg, len);
        }
    }
}

int main()
{
    initBoard();

    SX1278::Config config;
    SX1278::Error err;

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    printConfig(config);
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    printf("\n[sx1278] Initialization complete!\n");

    // God please forgive me
    // FIXME(davide.mor): ABSOLUTELY fix this
    miosix::Thread::sleep(10000);
    miosix::reboot();

    return 0;
}
