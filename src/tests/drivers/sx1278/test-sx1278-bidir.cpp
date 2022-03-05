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

#include <radio/SX1278/SX1278.h>
#include <drivers/interrupt/external_interrupts.h>

#include <cstring>
#include <thread>

#include "test-sx1278-core.h"

using namespace Boardcore;
using namespace miosix;

/**
 * Connection diagram:
 * sx1278[0]:nss  -> stm32:pa1
 * sx1278[0]:dio0 -> stm32:pc15
 * sx1278[0]:mosi -> stm32:pc12 (SPI3_MOSI)
 * sx1278[0]:miso -> stm32:pc11 (SPI3_MISO)
 * sx1278[0]:sck  -> stm32:pc10 (SPI3_SCK)

 * sx1278[1]:nss  -> stm32:pa2
 * sx1278[1]:dio0 -> stm32:pc0
 * sx1278[1]:mosi -> stm32:pb15 (SPI2_MOSI)
 * sx1278[1]:miso -> stm32:pb14 (SPI2_MISO)
 * sx1278[1]:sck  -> stm32:pb13 (SPI2_SCK)
 */

SPIBus bus0(SPI3);
SPIBus bus1(SPI2);

GpioPin sck0(GPIOC_BASE, 10);
GpioPin miso0(GPIOC_BASE, 11);
GpioPin mosi0(GPIOC_BASE, 12);
GpioPin cs0(GPIOA_BASE, 1);
GpioPin dio0(GPIOC_BASE, 15);

GpioPin sck1(GPIOB_BASE, 13);
GpioPin miso1(GPIOB_BASE, 14);
GpioPin mosi1(GPIOB_BASE, 15);
GpioPin cs1(GPIOA_BASE, 2);
GpioPin dio1(GPIOC_BASE, 0);

SX1278 *sx1278[2] = {nullptr, nullptr};

void __attribute__((used)) EXTI15_IRQHandlerImpl()
{
    if (sx1278[0])
        sx1278[0]->handleDioIRQ();
}

void __attribute__((used)) EXTI0_IRQHandlerImpl()
{
    if (sx1278[1])
        sx1278[1]->handleDioIRQ();
}

/// Initialize stm32f407g board (sx1278[0] only)
void initBoard0()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI3
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
        RCC_SYNC();

        // Setup SPI pins
        sck0.mode(miosix::Mode::ALTERNATE);
        sck0.alternateFunction(6);
        miso0.mode(miosix::Mode::ALTERNATE);
        miso0.alternateFunction(6);
        mosi0.mode(miosix::Mode::ALTERNATE);
        mosi0.alternateFunction(6);

        cs0.mode(miosix::Mode::OUTPUT);
        dio0.mode(miosix::Mode::INPUT);
    }

    cs0.high();
    enableExternalInterrupt(dio0.getPort(), dio0.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

/// Initialize stm32f407g board (sx1278[1] only)
void initBoard1()
{
    {
        miosix::FastInterruptDisableLock dLock;

        // Enable SPI2
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
        RCC_SYNC();

        sck1.mode(miosix::Mode::ALTERNATE);
        sck1.alternateFunction(5);
        miso1.mode(miosix::Mode::ALTERNATE);
        miso1.alternateFunction(5);
        mosi1.mode(miosix::Mode::ALTERNATE);
        mosi1.alternateFunction(5);

        cs1.mode(miosix::Mode::OUTPUT);
        dio1.mode(miosix::Mode::INPUT);
    }

    cs1.high();
    enableExternalInterrupt(dio1.getPort(), dio1.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

void recvLoop(int idx)
{
    char buf[256];
    while (1)
    {
        if (sx1278[idx]->receive((uint8_t *)buf, sizeof(buf)) != -1)
        {
            // Make sure there is a terminator somewhere
            buf[255] = 0;
            printf("[sx1278 @ %p] Received '%s'\n", sx1278[idx], buf);
        }
    }
}

void sendLoop(int idx, int interval, char *data)
{
    while (1)
    {
        miosix::Thread::sleep(interval);

        sx1278[idx]->send((uint8_t *)data, strlen(data) + 1);
        printf("[sx1278 @ %p] Sent '%s'\n", sx1278[idx], data);
    }
}

int main()
{
    initBoard0();
    initBoard1();

    // Run default configuration
    SX1278::Config config;
    SX1278::Error err;

    // Configure them
    sx1278[0] = new SX1278(bus0, cs0);

    printf("\n[sx1278] Configuring sx1278[0]...\n");
    printConfig(config);
    if ((err = sx1278[0]->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278[0]->init error: %s\n", stringFromErr(err));
        return -1;
    }

    sx1278[1] = new SX1278(bus1, cs1);

    printf("\n[sx1278] Configuring sx1278[1]...\n");
    printConfig(config);
    if ((err = sx1278[1]->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278[1]->init error: %s\n", stringFromErr(err));
        return -1;
    }

    // Spawn all threads
    std::thread send0(
        [=]() { sendLoop(0, 1000, const_cast<char *>("Hi from sx1278[0]!")); });
    std::thread send1(
        [=]() { sendLoop(1, 3333, const_cast<char *>("Hi from sx1278[1]!")); });

    std::thread recv0([]() { recvLoop(0); });
    std::thread recv1([]() { recvLoop(1); });

    printf("\n[sx1278] Initialization complete!\n");

    while (1)
        miosix::Thread::wait();

    return 0;
}
