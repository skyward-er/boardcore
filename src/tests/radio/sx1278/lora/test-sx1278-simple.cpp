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
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Lora.h>

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

// Uncomment the following line to enable Ebyte module
// #define IS_EBYTE

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#ifdef IS_EBYTE
using txen = Gpio<GPIOE_BASE, 4>;
using rxen = Gpio<GPIOD_BASE, 4>;
#endif

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

SX1278Lora *sx1278 = nullptr;

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void initBoard()
{
#ifdef IS_EBYTE
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif
}

int main()
{

    initBoard();

    SX1278Lora::Config config = {};
    config.power              = 10;
    config.ocp                = 0;
    config.coding_rate        = SX1278Lora::Config::Cr::CR_1;
    config.spreading_factor   = SX1278Lora::Config::Sf::SF_7;

    SX1278Lora::Error err;

    SPIBus bus(SX1278_SPI);

#ifdef IS_EBYTE
    std::unique_ptr<SX1278::ISX1278Frontend> frontend(
        new EbyteFrontend(txen::getPin(), rxen::getPin()));
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend(new RA01Frontend());
#endif

    sx1278 = new SX1278Lora(bus, cs::getPin(), dio0::getPin(), dio1::getPin(),
                            dio3::getPin(), SPI::ClockDivider::DIV_64,
                            std::move(frontend));

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error\n");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");

#ifdef ENABLE_TX
    int i = 0;

    while (1)
    {
        uint8_t msg[24] = {(uint8_t)i, 0xfe, 0xba, 0xbe};
        sx1278->send(msg, 24);

        for (int j = 0; j < 4; j++)
            printf("%x ", msg[j]);
        printf("\n");

        printf("[sx1278] sx1278->send successful!\n");
        miosix::Thread::sleep(1000);

        i += 1;
    }
#else
    while (1)
    {
        uint8_t msg[24] = {0};
        sx1278->receive(msg, 24);

        for (int j = 0; j < 4; j++)
            printf("%x ", msg[j]);

        printf("- RSSI: %f, SNR: %f\n", sx1278->getLastRxRssi(),
               sx1278->getLastRxSnr());
    }
#endif

    return 0;
}
