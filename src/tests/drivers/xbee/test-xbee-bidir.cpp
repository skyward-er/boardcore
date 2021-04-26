/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef RUN_SENDER
#define RUN_SENDER true
#endif
#ifndef RUN_RECEIVER
#define RUN_RECEIVER true
#endif

#include <miosix.h>

#include <cstdio>
#include <stdexcept>

#include "ActiveObject.h"
#include "XbeeTransceiver.h"
#include "drivers/Xbee/APIFramesLog.h"
#include "drivers/Xbee/ATCommands.h"
#include "drivers/Xbee/Xbee.h"
#include "drivers/interrupt/external_interrupts.h"
#include "logger/Logger.h"

using namespace miosix;

#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
#include "interfaces-impl/hwmapping.h"
using GpioMiso = miosix::interfaces::spi2::miso;
using GpioMosi = miosix::interfaces::spi2::mosi;
using GpioSck  = miosix::interfaces::spi2::sck;

using GpioCS   = xbee::cs;
using GpioATTN = xbee::attn;
using GpioRST  = xbee::reset;

using GpioLedLog = Gpio<GPIOG_BASE, 2>;

#define XBEE_SPI SPI2
#else
using GpioMiso = Gpio<GPIOB_BASE, 4>;
using GpioMosi = Gpio<GPIOA_BASE, 7>;
using GpioSck  = Gpio<GPIOA_BASE, 5>;

using GpioCS   = Gpio<GPIOC_BASE, 1>;
using GpioATTN = Gpio<GPIOE_BASE, 5>;
using GpioRST  = Gpio<GPIOE_BASE, 6>;

using GpioLedLog = Gpio<GPIOC_BASE, 13>;
#define XBEE_SPI SPI1
#endif

using GpioUserBtn = Gpio<GPIOA_BASE, 0>;

Xbee::Xbee* xbee_driver = nullptr;
Logger& logger          = Logger::instance();

#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
void __attribute__((used)) EXTI10_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI5_IRQHandlerImpl()
#endif
{
    if (xbee_driver != nullptr)
    {
        xbee_driver->handleATTNInterrupt();
    }
}

int getUserBtnValue()
{
#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
    return 0;
#else
    return GpioUserBtn::value();
#endif
}

void configure()
{
#ifndef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        // Set SPI pins to correct alternate mode
        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        GpioATTN::mode(Mode::INPUT);

        GpioLedLog::mode(Mode::OUTPUT);

        GpioUserBtn::mode(Mode::INPUT_PULL_DOWN);

        GpioCS::mode(Mode::OUTPUT);
    }

    GpioCS::high();
    GpioLedLog::low();
#endif

#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
    enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::FALLING_EDGE);
#else
    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::FALLING_EDGE);
#endif
}

void setupXbee(XbeeConfig config)
{
    if (xbee_driver)
    {
        if (config.data_rate_80k)
        {
            if (!Xbee::setDataRate(*xbee_driver, true))
            {
                TRACE("[main] Error setting xbee_driver data rate!\n");
            }
        }

        if (!config.freq_hop)
        {
            if (!Xbee::disableFrequencyHopping(*xbee_driver))
            {
                TRACE("[main] Error disabling frequency hop!\n");
            }
        }
    }
}

int main()
{
    XbeeConfig config;
    config.data_rate_80k = true;
    config.freq_hop      = true;

    config.packet_size   = 256;
    config.send_interval = 333;
    config.tx_enabled    = RUN_SENDER;
    config.timestamp     = getTick();

    configure();

    int filenum;
    try
    {
        filenum = logger.start();

        printf("\nLog file opened! (%s)\n\n",
               logger.getFileName(filenum).c_str());
    }
    catch (const std::runtime_error& err)
    {
        GpioLedLog::high();
        printf("\n!!!!!!Error opening log file!!!!!!!\n\n");
    }

    logger.log(config);

    SPIBus spi_bus(XBEE_SPI);
    SPIBusConfig cfg{};
    cfg.clock_div = SPIClockDivider::DIV16;

    GpioPin cs   = GpioCS::getPin();
    GpioPin attn = GpioATTN::getPin();
    GpioPin rst  = GpioRST::getPin();

    xbee_driver = new Xbee::Xbee(spi_bus, cfg, cs, attn, rst);

    setupXbee(config);

    // RandSendInterval intv(333, 1500);
    ConstSendInterval intv(config.send_interval);
    XbeeTransceiver* trans =
        new XbeeTransceiver(*xbee_driver, logger, intv, 256, 333);
    if (!config.tx_enabled)
    {
        trans->disableSender();
    }
    if (!RUN_RECEIVER)
    {
        trans->disableReceiver();
    }
    trans->start();

    while (getUserBtnValue() == 0)
    {
        long long loop_start = getTick();

        DataRateResult res_rcv = trans->getReceiver().getDataRate();
        DataRateResult res_snd = trans->getSender().getDataRate();

        TxData txd = trans->getSender().getTxData();
        RxData rxd = trans->getReceiver().getRxData();

        logger.log(xbee_driver->getStatus());
        logger.log(logger.getLogStats());

        long long tick = getTick();
        unsigned int h = tick / (1000 * 3600);
        unsigned int m = (tick - h * 1000 * 3600) / (1000 * 60);
        float s        = (tick - h * 1000 * 3600 - m * 1000 * 60) / 1000.0f;

        printf("%02u:%02u:%06.3f\n", h, m, s);
        if (RUN_SENDER)
        {
            printf("SND: int: %d, cnt: %d, tts: %u ms, pps: %.1f, fail: % d\n ",
                   txd.time_since_last_send,
                   txd.tx_success_counter + txd.tx_fail_counter,
                   txd.time_to_send, res_snd.packets_per_second,
                   txd.tx_fail_counter);
        }
        if (RUN_RECEIVER)
        {
            printf(
                "RCV: cnt: %d, last_rx: %lld ms,  RSSI: %d, dr: %.0f, pps: "
                "%.1f,"
                " pl: %.0f%%, lcnt: %u, fail: %u\n",
                rxd.rcv_count, rxd.last_packet_timestamp, rxd.RSSI,
                res_rcv.data_rate, res_rcv.packets_per_second,
                res_rcv.packet_loss * 100, rxd.packets_lost, rxd.rcv_errors);
        }
        printf("\n");

        Thread::sleepUntil(loop_start + 1000);
    }

    trans->stop();
    delete trans;
    delete xbee_driver;
    logger.stop();
    printf("Log closed.\n");

    for (;;)
    {
        GpioLedLog::high();
        Thread::sleep(50);
        GpioLedLog::low();
        Thread::sleep(5000);
    }
}