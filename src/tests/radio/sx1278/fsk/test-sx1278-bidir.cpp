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
#include <miosix.h>
#include <radio/SX1278/Ebyte.h>
#include <radio/SX1278/SX1278Fsk.h>

#include <cstring>
#include <thread>

#include "common.h"

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

using txen = Gpio<GPIOE_BASE, 4>;
using rxen = Gpio<GPIOD_BASE, 4>;

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI2_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

SX1278Fsk *sx1278 = nullptr;

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO0);
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO1);
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Fsk::Dio::DIO3);
}

void initBoard()
{
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();

    GpioPin dio0_pin = dio0::getPin();
    GpioPin dio1_pin = dio1::getPin();
    GpioPin dio3_pin = dio3::getPin();

    enableExternalInterrupt(dio0_pin.getPort(), dio0_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio1_pin.getPort(), dio1_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio3_pin.getPort(), dio3_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

void recvLoop()
{
    char buf[64];
    while (1)
    {
        ssize_t res =
            sx1278->receive(reinterpret_cast<uint8_t *>(buf), sizeof(buf));
        if (res != -1)
        {
            // Make sure there is a terminator somewhere
            buf[res] = 0;
            printf("[sx1278] Received '%s', power: %.2f\n", buf,
                   sx1278->getLastRxRssi());
        }
    }
}

void sendLoop(int interval, const char *data)
{
    char buf[64];
    strncpy(buf, data, sizeof(buf) - 1);

    while (1)
    {
        miosix::Thread::sleep(interval);

        sx1278->send(reinterpret_cast<uint8_t *>(buf), strlen(buf) + 1);
        printf("[sx1278] Sent '%s'\n", buf);
    }
}

int main()
{
    initBoard();

    // Run default configuration
    SX1278Fsk::Config config;
    config.power = 2;

    SX1278Fsk::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    sx1278 = new SX1278Fsk(bus, cs, SPI::ClockDivider::DIV_64);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Fsk::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));
        return -1;
    }

    printConfig(config);

    printf("\n[sx1278] Initialization complete!\n");

    // Spawn all threads
    std::thread send([]() { sendLoop(1000, "Sample radio message"); });
    std::thread recv([]() { recvLoop(); });

    // sx1278->debugDumpRegisters();

    while (1)
        miosix::Thread::wait();

    return 0;
}
