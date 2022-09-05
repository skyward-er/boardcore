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

#include "common.h"

using namespace Boardcore;
using namespace miosix;

SPIBus bus(SPI4);

GpioPin sck(GPIOE_BASE, 2);
GpioPin miso(GPIOE_BASE, 5);
GpioPin mosi(GPIOE_BASE, 6);
GpioPin cs(GPIOC_BASE, 1);
GpioPin dio(GPIOF_BASE, 10);

SX1278* sx1278 = nullptr;

void __attribute__((used)) EXTI10_IRQHandlerImpl()
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
        RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;  // Enable SPI4 bus
        RCC_SYNC();

        // Setup SPI pins
        sck.mode(miosix::Mode::ALTERNATE);
        sck.alternateFunction(5);
        miso.mode(miosix::Mode::ALTERNATE);
        miso.alternateFunction(5);
        mosi.mode(miosix::Mode::ALTERNATE);
        mosi.alternateFunction(5);

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
    // I create a GPIO with the onboard led to tell the user that
    // a package is being sent
    miosix::GpioPin led(GPIOG_BASE, 13);
    led.mode(miosix::Mode::OUTPUT);
    led.low();

    uint8_t msg[63];
    while (1)
    {
        auto serial = miosix::DefaultConsole::instance().get();
        int len     = serial->readBlock(msg, sizeof(msg), 0);
        if (len > 0)
        {
            led.high();
            sx1278->send(msg, len);
            led.low();
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

    printConfig(config);

    printf("\n[sx1278] Initialization complete!\n");

    std::thread recv([]() { recvLoop(); });
    std::thread send([]() { sendLoop(); });

    for (;;)
    {
        miosix::Thread::sleep(100);
    }

    // God please forgive me
    // FIXME(davide.mor): ABSOLUTELY fix this
    // miosix::Thread::sleep(20000);
    // miosix::reboot();

    return 0;
}
