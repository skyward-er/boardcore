/* Copyright (c) 2022 Skyward Experimental Rocketry
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

// SX1278 includes
#include <radio/SX1278/Ebyte.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SX1278/SX1278Lora.h>

#include <thread>

using namespace Boardcore;
using namespace miosix;

// Uncomment the following line to enable Lora mode
// Or use SBS to define it for you
// #define SX1278_IS_LORA

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

// Uncomment the following line to enable Ebyte module
// #define SX1278_IS_EBYTE

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#ifdef SX1278_IS_EBYTE
using txen = Gpio<GPIOE_BASE, 4>;
using rxen = Gpio<GPIOD_BASE, 4>;
#endif

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI2_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

#ifdef SX1278_IS_LORA
static constexpr size_t SX1278_MTU = SX1278Lora::MTU;
SX1278Lora *sx1278                 = nullptr;
#else
static constexpr size_t SX1278_MTU = SX1278Fsk::MTU;
SX1278Fsk *sx1278                  = nullptr;
#endif

#ifdef SX1278_IRQ_DIO0
void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278::Dio::DIO0);
}
#endif

#ifdef SX1278_IRQ_DIO1
void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278::Dio::DIO1);
}
#endif

#ifdef SX1278_IRQ_DIO3
void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278::Dio::DIO3);
}
#endif

void initBoard()
{
#ifdef SX1278_IS_EBYTE
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif

#ifdef SX1278_IRQ_DIO0
    GpioPin dio0_pin = dio0::getPin();
    enableExternalInterrupt(dio0_pin.getPort(), dio0_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
#endif

#ifdef SX1278_IRQ_DIO1
    GpioPin dio1_pin = dio1::getPin();
    enableExternalInterrupt(dio1_pin.getPort(), dio1_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
#endif

#ifdef SX1278_IRQ_DIO3
    GpioPin dio3_pin = dio3::getPin();
    enableExternalInterrupt(dio3_pin.getPort(), dio3_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
#endif
}

void recvLoop()
{
    uint8_t msg[SX1278_MTU];
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
    uint8_t msg[SX1278_MTU];
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

    // Generic SPI configuration
    SPIBusConfig spi_config;
    spi_config.clockDivider = SPI::ClockDivider::DIV_64;
    spi_config.mode         = SPI::Mode::MODE_0;
    spi_config.bitOrder     = SPI::BitOrder::MSB_FIRST;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    SPISlave spi(bus, cs, spi_config);

#ifdef SX1278_IS_LORA
    // Run default configuration
    SX1278Lora::Config config;
    SX1278Lora::Error err;

#ifdef SX1278_IS_EBYTE
    sx1278 = new EbyteLora(spi, txen::getPin(), rxen::getPin());
#else
    sx1278 = new SX1278Lora(spi);
#endif

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error\n");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");
#else
    // Run default configuration
    SX1278Fsk::Config config;
    SX1278Fsk::Error err;

#ifdef SX1278_IS_EBYTE
    sx1278 = new EbyteFsk(spi, txen::getPin(), rxen::getPin());
#else
    sx1278 = new SX1278Fsk(spi);
#endif

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Fsk::Error::NONE)
    {
                         // FIXME: Why does clang-format put this line up here?
        printf("[sx1278] sx1278->init error\n");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");
#endif

    // Actually spawn threads
    std::thread send([]() { sendLoop(); });
    recvLoop();

    return 0;
}
