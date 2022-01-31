/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#ifndef RUN_SENDER
#define RUN_SENDER true
#endif
#ifndef RUN_RECEIVER
#define RUN_RECEIVER true
#endif

#include <drivers/Xbee/APIFramesLog.h>
#include <drivers/Xbee/ATCommands.h>
#include <drivers/Xbee/Xbee.h>
#include <drivers/interrupt/external_interrupts.h>
#include <logger/Logger.h>
#include <miosix.h>

#include <cstdio>
#include <stdexcept>

#include "ActiveObject.h"
#include "XbeeTransceiver.h"

using namespace Boardcore;
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

Xbee::Xbee* xbeeDriver = nullptr;
Logger& logger         = Logger::getInstance();

#ifdef _BOARD_STM32F429ZI_SKYWARD_DEATHST_X
void __attribute__((used)) EXTI10_IRQHandlerImpl()
#else
void __attribute__((used)) EXTI5_IRQHandlerImpl()
#endif
{
    if (xbeeDriver != nullptr)
    {
        xbeeDriver->handleATTNInterrupt();
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
    if (xbeeDriver)
    {
        if (config.dataRate80k)
        {
            if (!Xbee::setDataRate(*xbeeDriver, true))
            {
                TRACE("[main] Error setting xbeeDriver data rate!\n");
            }
        }

        if (!config.freqHop)
        {
            if (!Xbee::disableFrequencyHopping(*xbeeDriver))
            {
                TRACE("[main] Error disabling frequency hop!\n");
            }
        }
    }
}

int main()
{
    XbeeConfig config;
    config.dataRate80k = true;
    config.freqHop     = true;

    config.packetSize   = 256;
    config.sendInterval = 333;
    config.txEnabled    = RUN_SENDER;
    config.timestamp    = getTick();

    configure();

    try
    {
        logger.start();

        printf("\nLog file opened! (%s)\n\n",
               logger.getCurrentFileName().c_str());
    }
    catch (const std::runtime_error& err)
    {
        GpioLedLog::high();
        printf("\n!!!!!!Error opening log file!!!!!!!\n\n");
    }

    logger.log(config);

    SPIBus spiBus(XBEE_SPI);
    SPIBusConfig cfg{};
    cfg.clockDivider = SPI::ClockDivider::DIV_16;

    GpioPin cs   = GpioCS::getPin();
    GpioPin attn = GpioATTN::getPin();
    GpioPin rst  = GpioRST::getPin();

    xbeeDriver = new Xbee::Xbee(spiBus, cfg, cs, attn, rst);

    setupXbee(config);

    // RandSendInterval intv(333, 1500);
    ConstSendInterval intv(config.sendInterval);
    XbeeTransceiver* trans =
        new XbeeTransceiver(*xbeeDriver, logger, intv, 256, 333);
    if (!config.txEnabled)
    {
        trans->disableSender();
    }
    if (!RUN_RECEIVER)
    {
        trans->disableReceiver();
    }
    trans->start();

    // cppcheck-suppress knownConditionTrueFalse
    while (getUserBtnValue() == 0)
    {
        long long loopStart = getTick();

        DataRateResult resRcv = trans->getReceiver().getDataRate();
        DataRateResult resSnd = trans->getSender().getDataRate();

        TxData txd = trans->getSender().getTxData();
        RxData rxd = trans->getReceiver().getRxData();

        logger.log(xbeeDriver->getStatus());
        logger.log(logger.getLoggerStats());

        long long tick = getTick();
        unsigned int h = tick / (1000 * 3600);
        unsigned int m = (tick - h * 1000 * 3600) / (1000 * 60);
        float s        = (tick - h * 1000 * 3600 - m * 1000 * 60) / 1000.0f;

        printf("%02u:%02u:%06.3f\n", h, m, s);
        if (RUN_SENDER)
        {
            printf("SND: int: %u, cnt: %u, tts: %u ms, pps: %.1f, fail: %u\n ",
                   txd.timeSinceLastSend,
                   txd.txSuccessCounter + txd.txFailCounter, txd.timeToSend,
                   resSnd.packetsPerSecond, txd.txFailCounter);
        }
        if (RUN_RECEIVER)
        {
            printf(
                "RCV: cnt: %u, last_rx: %lld ms,  RSSI: %d, dr: %.0f, pps: "
                "%.1f,"
                " pl: %.0f%%, lcnt: %u, fail: %u\n",
                rxd.rcvCount, rxd.lastPacketTimestamp, rxd.RSSI,
                resRcv.dataRate, resRcv.packetsPerSecond,
                resRcv.packetLoss * 100, rxd.packetsLost, rxd.rcvCount);
        }
        printf("\n");

        Thread::sleepUntil(loopStart + 1000);
    }

    trans->stop();
    delete trans;
    delete xbeeDriver;
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
