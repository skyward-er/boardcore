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
#include <radio/SX1278/SX1278.h>

#include <thread>

#include "common.h"
#include "gui/GUI.h"

// Include body of the test
#include "test-sx1278-bench.cpp"

using namespace Boardcore;
using namespace miosix;
using namespace mxgui;

#if defined _BOARD_STM32F429ZI_SKYWARD_GS_V2
#include "interfaces-impl/hwmapping.h"

using cs   = peripherals::ra01::pc13::cs;
using dio0 = peripherals::ra01::pc13::dio0;
using dio1 = peripherals::ra01::pc13::dio1;
using dio3 = peripherals::ra01::pc13::dio3;

using sck  = interfaces::spi4::sck;
using miso = interfaces::spi4::miso;
using mosi = interfaces::spi4::mosi;

#define SX1278_SPI SPI4

#define IRQ_DIO0 EXTI6_IRQHandlerImpl
#define IRQ_DIO1 EXTI5_IRQHandlerImpl
#define IRQ_DIO3 EXTI11_IRQHandlerImpl

#else
#error "Target not supported"
#endif

void __attribute__((used)) IRQ_DIO0()
{
    if (sx1278)
        sx1278->handleDioIRQ();
}

void __attribute__((used)) IRQ_DIO1()
{
    // if (sx1278)
    //     sx1278->handleDioIRQ();
}

void __attribute__((used)) IRQ_DIO3()
{
    // if (sx1278)
    //     sx1278->handleDioIRQ();
}

void initBoard()
{
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

GUI *gui = nullptr;

void initGUI()
{
    // TODO: This should be in bsp
    using GpioUserBtn = Gpio<GPIOA_BASE, 0>;
    GpioUserBtn::mode(Mode::INPUT_PULL_DOWN);

    gui = new GUI();

    ButtonHandler::getInstance().registerButtonCallback(
        GpioUserBtn::getPin(),
        [](auto event) { gui->screen_manager.onButtonEvent(event); });
}

int main()
{
    initBoard();
    initGUI();

    // Run default configuration
    SX1278::Config config = {
        .freq_rf  = 422075000,
        .freq_dev = 25000,
        .bitrate  = 19200,
        .rx_bw    = Boardcore::SX1278::RxBw::HZ_83300,
        .afc_bw   = Boardcore::SX1278::RxBw::HZ_125000,
        .ocp      = 120,
        .power    = 17,
        .shaping  = Boardcore::SX1278::Shaping::GAUSSIAN_BT_1_0,
        .dc_free  = Boardcore::SX1278::DcFree::WHITENING};
    SX1278::Error err;

    SPIBus bus(SX1278_SPI);
    GpioPin cs = cs::getPin();

    sx1278 = new SX1278(bus, cs);

    printf("\n[sx1278] Configuring sx1278...\n");
    if ((err = sx1278->init(config)) != SX1278::Error::NONE)
    {
        gui->stats_screen.updateError(err);
        printf("[sx1278] sx1278->init error: %s\n", stringFromErr(err));

        while (1)
            Thread::wait();
    }

    printConfig(config);
    gui->stats_screen.updateReady();

    // Initialize backgrounds threads
    spawnThreads();

    while (1)
    {
        StatsScreen::Data data = {
            stats.txBitrate(), stats.rxBitrate(), stats.corrupted_count,
            stats.sent_count,  stats.recv_count,  0.0f /* TODO: Packet loss */,
            stats.rssi};

        gui->stats_screen.updateStats(data);
        Thread::sleep(100);
    }
}
