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
#include <radio/SX1278/SX1278.h>

#include <thread>

#include "common.h"

// Include body of the test
#include "test-sx1278-bench.cpp"

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
#include "interfaces-impl/hwmapping.h"

#if 1  // use ra01
using cs   = peripherals::ra01::cs;
using dio0 = peripherals::ra01::dio0;
#else
using cs   = peripherals::sx127x::cs;
using dio0 = peripherals::sx127x::dio0;
#endif

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#define SX1278_SPI SPI4

#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
#include "interfaces-impl/hwmapping.h"

using cs   = sensors::sx127x::cs;
using dio0 = sensors::sx127x::dio0;

using sck  = interfaces::spi5::sck;
using miso = interfaces::spi5::miso;
using mosi = interfaces::spi5::mosi;

#define SX1278_SPI SPI5

#else
#error "Target not supported"
#endif

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
void __attribute__((used)) EXTI6_IRQHandlerImpl()
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
void __attribute__((used)) EXTI10_IRQHandlerImpl()
#else
#error "Target not supported"
#endif
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void initBoard()
{
#if defined _BOARD_STM32F429ZI_SKYWARD_GS
    enableExternalInterrupt(GPIOF_BASE, 6, InterruptTrigger::RISING_EDGE);
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
    enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::RISING_EDGE);
#else
#error "Target not supported"
#endif
}

int main()
{
    initBoard();

    // Run default configuration
    SX1278::Config config;
    SX1278::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    printConfig(config);
    printf("\n[sx1278] Initialization complete!\n");

    // Initialize backgrounds threads
    spawnThreads();

    while (1)
    {
        printf(
            "\n[sx1278] Stats:\n"
            "Tx bitrate:        %.2f kb/s\n"
            "Packet sent:       %d\n"
            "Rx bitrate:        %.2f kb/s\n"
            "Packet received:   %d\n"
            "Corrupted packets: %d\n"
            "Packet loss:       %.2f %%\n"
            "RSSI:              %.2f dBm\n"
            "FEI:               %.2f dBm\n",
            static_cast<float>(stats.txBitrate()) / 1000.0f, stats.sent_count,
            static_cast<float>(stats.rxBitrate()) / 1000.0f, stats.recv_count,
            stats.corrupted_count, 0.0f /* TODO: Packet loss */, stats.rssi,
            stats.fei);

        miosix::Thread::sleep(2000);
    }

    return 0;
}
