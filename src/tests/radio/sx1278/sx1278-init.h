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

#pragma once

#include <drivers/interrupt/external_interrupts.h>
#include <miosix.h>

// SX1278 includes
#include <radio/SX1278/SX1278Frontends.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <radio/SX1278/SX1278Lora.h>

// Uncomment the following line to enable Lora mode
// Or use SBS to define it for you
// #define SX1278_IS_LORA

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

// Uncomment the following line to enable Ebyte module
// #define SX1278_IS_EBYTE
// Uncomment the following line to ebable Skyward433 module
// #define SX1278_IS_SKYWARD433

using cs   = miosix::peripherals::ra01::pc13::cs;
using dio0 = miosix::peripherals::ra01::pc13::dio0;
using dio1 = miosix::peripherals::ra01::pc13::dio1;
using dio3 = miosix::peripherals::ra01::pc13::dio3;

using sck  = miosix::interfaces::spi4::sck;
using miso = miosix::interfaces::spi4::miso;
using mosi = miosix::interfaces::spi4::mosi;

#ifdef SX1278_IS_EBYTE
using txen = miosix::Gpio<GPIOE_BASE, 4>;
using rxen = miosix::Gpio<GPIOD_BASE, 4>;
#endif

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI6_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI11_IRQHandlerImpl

#elif defined _BOARD_STM32F429ZI_SKYWARD_RIG
#include "interfaces-impl/hwmapping.h"

#define SX1278_IS_EBYTE

using cs   = miosix::radio::cs;
using dio0 = miosix::radio::dio0;
using dio1 = miosix::radio::dio1;
using dio3 = miosix::radio::dio3;

using sck  = miosix::radio::sck;
using miso = miosix::radio::miso;
using mosi = miosix::radio::mosi;

using txen                         = miosix::radio::txEn;
using rxen                         = miosix::radio::rxEn;

#define SX1278_SPI SPI4

#define SX1278_IRQ_DIO0 EXTI5_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI12_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI13_IRQHandlerImpl

#elif defined _BOARD_STM32F767ZI_GEMINI_GS
#include "interfaces-impl/hwmapping.h"

#define SX1278_IS_SKYWARD433
// #define SX1278_IS_EBYTE

// Comment to use SX1278_2
#define SX1278_1

#ifdef SX1278_1
using cs   = miosix::radio1::cs;
using dio0 = miosix::radio1::dio0;
using dio1 = miosix::radio1::dio1;
using dio3 = miosix::radio1::dio3;

using sck  = miosix::radio1::spi::sck;
using miso = miosix::radio1::spi::miso;
using mosi = miosix::radio1::spi::mosi;

using txen = miosix::radio1::txen;
using rxen = miosix::radio1::rxen;

#define SX1278_NRST
using rst  = miosix::radio1::nrst;

#define SX1278_SPI MIOSIX_RADIO1_SPI

#define SX1278_IRQ_DIO0 MIOSIX_RADIO1_DIO0_IRQ
#define SX1278_IRQ_DIO1 MIOSIX_RADIO1_DIO1_IRQ
#define SX1278_IRQ_DIO3 MIOSIX_RADIO1_DIO3_IRQ
#else
using cs   = miosix::radio2::cs;
using dio0 = miosix::radio2::dio0;
using dio1 = miosix::radio2::dio1;
using dio3 = miosix::radio2::dio3;

using sck  = miosix::radio2::spi::sck;
using miso = miosix::radio2::spi::miso;
using mosi = miosix::radio2::spi::mosi;

using txen = miosix::radio2::txen;
using rxen = miosix::radio2::rxen;

#define SX1278_NRST
using rst  = miosix::radio2::nrst;

#define SX1278_SPI MIOSIX_RADIO2_SPI

#define SX1278_IRQ_DIO0 MIOSIX_RADIO2_DIO0_IRQ
#define SX1278_IRQ_DIO1 MIOSIX_RADIO2_DIO1_IRQ
#define SX1278_IRQ_DIO3 MIOSIX_RADIO2_DIO3_IRQ
#endif

#else
#error "Target not supported"
#endif

#ifdef SX1278_IS_LORA
static constexpr size_t SX1278_MTU = Boardcore::SX1278Lora::MTU;
Boardcore::SX1278Lora *sx1278      = nullptr;
#else
static constexpr size_t SX1278_MTU = Boardcore::SX1278Fsk::MTU;
Boardcore::SX1278Fsk *sx1278       = nullptr;
#endif

volatile int dio0_cnt = 0;
volatile int dio1_cnt = 0;
volatile int dio3_cnt = 0;

#ifdef SX1278_IRQ_DIO0
void __attribute__((used)) SX1278_IRQ_DIO0()
{
    dio0_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

#ifdef SX1278_IRQ_DIO1
void __attribute__((used)) SX1278_IRQ_DIO1()
{
    dio1_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

#ifdef SX1278_IRQ_DIO3
void __attribute__((used)) SX1278_IRQ_DIO3()
{
    dio3_cnt++;
    if (sx1278)
        sx1278->handleDioIRQ();
}
#endif

void initBoard()
{
#ifdef SX1278_IS_EBYTE
    rxen::mode(miosix::Mode::OUTPUT);
    txen::mode(miosix::Mode::OUTPUT);
    rxen::low();
    txen::low();
#endif

#ifdef SX1278_NRST
    rst::mode(miosix::Mode::OUTPUT);
    rst::high();
#endif
}

Boardcore::SPIBus sx1278_bus(SX1278_SPI);

bool initRadio()
{
    // Initialize frontend (if any)
#if defined SX1278_IS_EBYTE
    printf("[sx1278] Confuring Ebyte frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
        new Boardcore::EbyteFrontend(txen::getPin(), rxen::getPin()));
#elif defined SX1278_IS_SKYWARD433
    printf("[sx1278] Confuring Skyward 433 frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
              new Boardcore::Skyward433Frontend());
#else
    printf("[sx1278] Confuring RA01 frontend...\n");
    std::unique_ptr<Boardcore::SX1278::ISX1278Frontend> frontend(
         new Boardcore::RA01Frontend());
#endif

    // Initialize actual radio driver
#ifdef SX1278_IS_LORA
    // Run default configuration
    Boardcore::SX1278Lora::Config config;
    Boardcore::SX1278Lora::Error err;

    sx1278 = new Boardcore::SX1278Lora(sx1278_bus, cs::getPin(), dio0::getPin(),
                                       dio1::getPin(), dio3::getPin(),
                                       Boardcore::SPI::ClockDivider::DIV_256,
                                       std::move(frontend));

    printf("\n[sx1278] Configuring sx1278 lora...\n");
    if ((err = sx1278->init(config)) != Boardcore::SX1278Lora::Error::NONE)
    {
        printf("[sx1278] sx1278->init error\n");
        return false;
    }

    printf("\n[sx1278] Initialization complete!\n");
#else
    // Run default configuration
    Boardcore::SX1278Fsk::Config config;
    Boardcore::SX1278Fsk::Error err;

    sx1278 = new Boardcore::SX1278Fsk(sx1278_bus, cs::getPin(), dio0::getPin(),
                                            dio1::getPin(), dio3::getPin(),
                                            Boardcore::SPI::ClockDivider::DIV_256,
                                            std::move(frontend));

    printf("\n[sx1278] Configuring sx1278 fsk...\n");
    if ((err = sx1278->init(config)) != Boardcore::SX1278Fsk::Error::NONE)
    {
              // FIXME: Why does clang-format put this line up here?
        printf("[sx1278] sx1278->init error\n");
        return false;
    }

    printf("\n[sx1278] Initialization complete!\n");
#endif

    return true;
}
