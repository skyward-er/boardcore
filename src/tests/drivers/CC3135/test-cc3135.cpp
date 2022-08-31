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
#include <radio/CC3135/CC3135.h>

#include <thread>

using namespace Boardcore;
using namespace miosix;

#define CHECK(expr)                                       \
    do                                                    \
    {                                                     \
        Boardcore::CC3135::Error result = (expr);         \
        if (result != Boardcore::CC3135::Error::NO_ERROR) \
            printf("[cc3135] NWP error!\n");              \
    } while (0)

/*
Connection diagram:
cc3135:CC_SPI_CS   -> stm32:pa1
cc3135:CC_nHIB     -> stm32:pc14
cc3135:CC_IRQ      -> stm32:pc15
cc3135:CC_SPI_DIN  -> stm32:pc12 (SPI3_MOSI)
cc3135:CC_SPI_DOUT -> stm32:pc11 (SPI3_MISO)
cc3135:CC_SPI_CLK  -> stm32:pc10 (SPI3_SCK)
*/

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
#include "interfaces-impl/hwmapping.h"
using sck  = interfaces::spi1::sck;
using miso = interfaces::spi1::miso;
using mosi = interfaces::spi1::mosi;
using cs   = peripherals::cc3135::cs;
using irq  = peripherals::cc3135::intr;
using hib  = peripherals::cc3135::hib;

#define CC3135_SPI SPI1
#define CC3135_HIB
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
#include "interfaces-impl/hwmapping.h"
using sck  = interfaces::spi6::sck;
using miso = interfaces::spi6::miso;
using mosi = interfaces::spi6::mosi;
using cs   = sensors::cc3135::cs;
using irq  = sensors::cc3135::intr;

#define CC3135_SPI SPI6
#else

// Use UART
#if 0
using tx  = Gpio<GPIOA_BASE, 2>;
using rx  = Gpio<GPIOA_BASE, 3>;
using irq = Gpio<GPIOA_BASE, 4>;
using hib = Gpio<GPIOA_BASE, 5>;

#define CC3135_UART USART2
// #define CC3135_HIB
#else
using sck  = Gpio<GPIOA_BASE, 5>;
using miso = Gpio<GPIOB_BASE, 4>;
using mosi = Gpio<GPIOA_BASE, 7>;
using cs   = Gpio<GPIOD_BASE, 4>;
using irq  = Gpio<GPIOA_BASE, 4>;
using hib  = Gpio<GPIOA_BASE, 6>;

#define CC3135_SPI SPI1
#define CC3135_HIB
#endif

#endif

CC3135 *cc3135 = nullptr;

volatile unsigned int IRQ_COUNT = 0;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS
void __attribute__((used)) EXTI5_IRQHandlerImpl()
#elif defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3
void __attribute__((used)) EXTI10_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI4_IRQHandlerImpl()
#endif
{
    IRQ_COUNT += 1;
    if (cc3135)
        cc3135->handleIrq();
}

void initBoard()
{
#if (defined CC3135_HIB && !defined _BOARD_STM32F429ZI_SKYWARD_GS && \
     !defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3)
    {
        miosix::FastInterruptDisableLock dLock;

        hib::mode(miosix::Mode::OUTPUT);
    }

    hib::high();
#endif

#ifdef CC3135_UART
    {
        miosix::FastInterruptDisableLock dLock;

        irq::mode(miosix::Mode::INPUT);
        tx::mode(miosix::Mode::ALTERNATE);
        tx::alternateFunction(7);
        rx::mode(miosix::Mode::ALTERNATE);
        rx::alternateFunction(7);
    }
#endif

#if (defined CC3135_SPI && !defined _BOARD_STM32F429ZI_SKYWARD_GS && \
     !defined _BOARD_STM32F429ZI_SKYWARD_DEATHST_V3)
    {
        miosix::FastInterruptDisableLock dLock;

        miso::mode(miosix::Mode::ALTERNATE);
        miso::alternateFunction(5);
        mosi::mode(miosix::Mode::ALTERNATE);
        mosi::alternateFunction(5);
        sck::mode(miosix::Mode::ALTERNATE);
        sck::alternateFunction(5);
        irq::mode(miosix::Mode::INPUT);
        cs::mode(miosix::Mode::OUTPUT);
    }

    cs::high();
#endif

    auto irq_pin = irq::getPin();
    enableExternalInterrupt(irq_pin.getPort(), irq_pin.getNumber(),
                            InterruptTrigger::RISING_EDGE);
}

int main()
{
    // IRQ watcher thread
    std::thread _watcher(
        []()
        {
            size_t last = -1;
            while (1)
            {
                if (last != IRQ_COUNT)
                {
                    printf("[cc3135] IRQ: %u\n", IRQ_COUNT);
                    last = IRQ_COUNT;
                }

                // Sleep to avoid CPU hogging
                Thread::sleep(10);
            }
        });

    initBoard();

    // For some reason, putting this before SPI peripheral activation makes it
    // more reliable
#ifdef CC3135_HIB
    // Reset CC3135
    hib::low();
    Thread::sleep(100);
    hib::high();
#endif

#ifdef CC3135_SPI
    SPIBus bus(CC3135_SPI);
    GpioPin cs_pin = cs::getPin();

    SPIBusConfig config = {};
    config.clockDivider = SPI::ClockDivider::DIV_64;
    config.mode         = SPI::Mode::MODE_0;
    config.bitOrder     = SPI::BitOrder::MSB_FIRST;

    std::unique_ptr<ICC3135Iface> iface(new CC3135Spi(bus, cs_pin, config));
#endif

#ifdef CC3135_UART
    std::unique_ptr<ICC3135Iface> iface(new CC3135Uart(CC3135_UART));
#endif

    printf("[cc3135] Initializing...\n");
    cc3135 = new CC3135(std::move(iface));

    auto result = cc3135->init(true);
    if (result != CC3135::Error::NO_ERROR)
    {
        printf("[cc3135] Failed to start cc3135 (error: %s), retrying...\n",
               CC3135::errorToStr(result));
        Thread::sleep(1000);

        miosix::reboot();
    }

    // cc3135->prepareForReset();
    // cc3135->setApChannel(100);

    printf("[cc3135] Initialization complete!\n");

    CC3135Defs::DeviceVersion version = {};
    CHECK(cc3135->getVersion(version));
    printf(
        "[cc3135] Chip Id: %lx\n"
        "[cc3135] Fw version: %u.%u.%u.%u\n"
        "[cc3135] Phy version: %u.%u.%u.%u\n"
        "[cc3135] Nwp version: %u.%u.%u.%u\n"
        "[cc3135] Rom version: %x\n",
        version.chip_id, version.fw_version[0], version.fw_version[1],
        version.fw_version[2], version.fw_version[3], version.phy_version[0],
        version.phy_version[1], version.phy_version[2], version.phy_version[3],
        version.nwp_version[0], version.nwp_version[1], version.nwp_version[2],
        version.nwp_version[3], version.rom_version);

    while (true)
    {
        // cc3135->dummySend();
        // cc3135->startRecv();

        // CHECK(cc3135->getVersion(version));
        // printf(
        //     "[cc3135] Chip Id: %lx\n"
        //     "[cc3135] Fw version: %u.%u.%u.%u\n"
        //     "[cc3135] Phy version: %u.%u.%u.%u\n"
        //     "[cc3135] Nwp version: %u.%u.%u.%u\n"
        //     "[cc3135] Rom version: %x\n",
        //     version.chip_id, version.fw_version[0], version.fw_version[1],
        //     version.fw_version[2], version.fw_version[3],
        //     version.phy_version[0], version.phy_version[1],
        //     version.phy_version[2], version.phy_version[3],
        //     version.nwp_version[0], version.nwp_version[1],
        //     version.nwp_version[2], version.nwp_version[3],
        //     version.rom_version);

        printf("[cc3135] Looping\n");

        Thread::sleep(1000);
    }

    while (true)
        ;

    // Thread::sleep(3000);
    // miosix::reboot();
}
