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
#include <radio/SX1278/SX1278Lora.h>

#include <cstring>
#include <thread>

using namespace Boardcore;
using namespace miosix;

#if defined _BOARD_STM32F429ZI_SKYWARD_GROUNDSTATION_NOKIA
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
#define SX1278_IRQ_DIO1 EXTI2_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

SX1278Lora *sx1278 = nullptr;

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Lora::Dio::DIO0);
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Lora::Dio::DIO1);
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    if (sx1278)
        sx1278->handleDioIRQ(SX1278Lora::Dio::DIO3);
}

void initBoard()
{
#ifdef IS_EBYTE
    rxen::mode(Mode::OUTPUT);
    txen::mode(Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif

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

bool isByteBuf(uint8_t *buf, ssize_t len)
{
    for (ssize_t i = 0; i < len; i++)
    {
        if (!isprint(buf[i]))
            return true;
    }

    return false;
}

void formatByteBuf(uint8_t *buf, ssize_t len, char *out)
{
    for (ssize_t i = 0; i < len; i++)
    {
        if (i == 0)
            out += sprintf(out, "%02X", buf[i]);
        else
            out += sprintf(out, ":%02X", buf[i]);
    }

    // Put terminator at the end
    *out = '\0';
}

void recvLoop()
{
    uint8_t buf[SX1278Lora::MTU];
    while (1)
    {
        ssize_t res = sx1278->receive(buf, sizeof(buf));
        if (res != -1)
        {
            char msg[512] = {0};
            if (isByteBuf(buf, res))
            {
                // This is a byte buffer, print it as an hexadecimal string
                formatByteBuf(buf, res, msg);
            }
            else
            {
                memcpy(msg, buf, res);
                msg[res + 1] = '\0';
            }

            // Make sure there is a terminator somewhere
            buf[res] = 0;
            printf("[sx1278] Received '%s', power: %.2f, snr: %.2f\n", msg,
                   sx1278->getLastRxRssi(), sx1278->getLastRxSnr());
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

        sx1278->send(reinterpret_cast<uint8_t *>(buf), strlen(buf));
        printf("[sx1278] Sent '%s'\n", buf);
    }
}

int main()
{
    initBoard();

    SX1278Lora::Config config = {};
    config.pa_boost           = true;
    config.power              = 10;
    config.ocp                = 0;
    config.coding_rate        = SX1278Lora::Config::Cr::CR_4;
    config.spreading_factor   = SX1278Lora::Config::Sf::SF_12;

    SX1278Lora::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    SPIBusConfig spi_config = {};
    spi_config.clockDivider = SPI::ClockDivider::DIV_64;
    spi_config.mode         = SPI::Mode::MODE_0;
    spi_config.bitOrder     = SPI::Order::MSB_FIRST;
    spi_config.byteOrder    = SPI::Order::MSB_FIRST;
    spi_config.writeBit     = SPI::WriteBit::INVERTED;

#ifdef IS_EBYTE
    sx1278 = new EbyteLora(SPISlave(bus, cs, spi_config), txen::getPin(),
                           rxen::getPin());
#else
    sx1278 = new SX1278Lora(SPISlave(bus, cs, spi_config));
#endif

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error: %s\n", "TODO");
        return -1;
    }

    printf("\n[sx1278] Initialization complete!\n");

    // Spawn all threads
    std::thread send([]() { sendLoop(3333, "Sample radio message"); });
    std::thread recv([]() { recvLoop(); });

    // sx1278->debugDumpRegisters();

    while (1)
        miosix::Thread::wait();

    return 0;
}
