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

#include <drivers/Xbee/APIFramesLog.h>
#include <drivers/Xbee/ATCommands.h>
#include <drivers/Xbee/Xbee.h>
#include <drivers/interrupt/external_interrupts.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <mxgui/display.h>
#include <utils/ButtonHandler.h>

#include <array>
#include <cstdio>
#include <functional>
#include <stdexcept>

#include "ActiveObject.h"
#include "Mark.h"
#include "XbeeTransceiver.h"
#include "gui/XbeeGui.h"

using namespace Boardcore;
using namespace miosix;
using namespace mxgui;
using namespace std::placeholders;

using std::array;
using std::bind;
using std::to_string;

/// Pin definitions
using GpioMiso = Gpio<GPIOB_BASE, 4>;
using GpioMosi = Gpio<GPIOA_BASE, 7>;
using GpioSck  = Gpio<GPIOA_BASE, 5>;

using GpioCS   = Gpio<GPIOC_BASE, 1>;
using GpioATTN = Gpio<GPIOE_BASE, 5>;
using GpioRST  = Gpio<GPIOE_BASE, 6>;

using GpioUserBtn = Gpio<GPIOA_BASE, 0>;
using GpioLedLog  = Gpio<GPIOC_BASE, 13>;

// Forward dec
void onStartButtonClick(View* btn, Interaction action);
void onEnergyButtonClick(View* btn, Interaction action);
void onMarkButtonClick(View* btn, Interaction action);
void onStopButtonClick(View* btn, Interaction action);
void startTransceiver(XbeeConfig config);
void setupXbee(XbeeConfig config);
void configure();

// Global variables
Logger& logger   = Logger::getInstance();
Xbee::Xbee* xbee = nullptr;
ConstSendInterval sndInt{0};
XbeeTransceiver* trans = nullptr;
XbeeGUI* gui;
ButtonHandler<GpioUserBtn>* btnHandler;

unsigned int markCounter = 1;

/**
 * @brief Activeobject to perform an energy detect scan as frequently as
 * possible
 */
class EnergyScanner : public ActiveObject
{
protected:
    void run() override
    {
        while (!shouldStop())
        {
            Xbee::ATCommandResponseFrame response;

            uint8_t duration = 0xFF;

            if (xbee->sendATCommand("ED", &response, &duration, 1, 1000) &&
                response.getCommandDataLength() == 30)
            {
                array<int, 30> scan;

                for (uint16_t i = 0; i < response.getCommandDataLength(); i++)
                {
                    scan[i] = -(int)(*(response.getCommandDataPointer() + i));
                }

                gui->screenEnergy.updateScan(scan);

                EnergyScanData data{getTick(), scan};
                logger.log(data);
            }
        }
    }
} energyScanner;

int main()
{
    // Hardware
    configure();

    // SD
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

    // XBee
    SPIBus spiBus(SPI1);
    SPIBusConfig cfg{};
    cfg.clockDivider = SPI::ClockDivider::DIV_16;

    GpioPin cs   = GpioCS::getPin();
    GpioPin attn = GpioATTN::getPin();
    GpioPin rst  = GpioRST::getPin();

    xbee = new Xbee::Xbee(spiBus, cfg, cs, attn, rst);

    // GUI
    gui = new XbeeGUI();

    btnHandler = new ButtonHandler<GpioUserBtn>(
        0, bind(&ScreenManager::onButtonPress, &gui->screenManager, _2));

    btnHandler->start();

    gui->screenConfig.btnStart.addOnInteractionListener(onStartButtonClick);
    gui->screenConfig.btnEnergy.addOnInteractionListener(onEnergyButtonClick);

    gui->screenStatus.btnStop.addOnInteractionListener(onStopButtonClick);
    gui->screenStatus.btnMark.addOnInteractionListener(onMarkButtonClick);

    gui->screenEnergy.btnStop.addOnInteractionListener(onStopButtonClick);
    gui->screenEnergy.btnMark.addOnInteractionListener(onMarkButtonClick);
    gui->screenEnergy.btnReset.addOnInteractionListener(
        [&](View* d, Interaction action)
        {
            UNUSED(d);
            if (action == Interaction::CLICK)
                gui->screenEnergy.resetStats();
        });

    gui->screenEnd.tvF.addOnInteractionListener(
        [&](View* d, Interaction action)
        {
            UNUSED(d);
            if (action == Interaction::CLICK)
                gui->screenManager.showScreen(XbeeGUI::SCREEN_RESPECT);
        });

    gui->screenEnd.tvReset.addOnInteractionListener(
        [&](View* d, Interaction action)
        {
            UNUSED(d);
            if (action == Interaction::CLICK)
                miosix::reboot();
        });

    // Main loop: updates the information in the GUI
    for (;;)
    {
        long long start = getTick();
        // Update display values
        switch (gui->screenManager.getScreen())
        {
            case XbeeGUI::SCREEN_CONFIG:
                gui->screenConfig.updateLogStatus(logger);
                break;
            case XbeeGUI::SCREEN_STATUS:
                if (trans && xbee)
                {
                    gui->screenStatus.updateXbeeStatus(
                        trans->getReceiver().getDataRate(),
                        trans->getSender().getDataRate(),
                        trans->getSender().getTxData(),
                        trans->getReceiver().getRxData(), xbee->getStatus());

                    logger.log(xbee->getStatus());
                }

                gui->screenStatus.updateLogStatus(logger);
                break;
            case XbeeGUI::SCREEN_ENERGYSCAN:
                gui->screenEnergy.updateLogStatus(logger);
                break;
            default:
                break;
        }

        logger.log(logger.getLoggerStats());
        Thread::sleepUntil(start + 500);
    }
}

void onStartButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {

        XbeeConfig cfg = gui->screenConfig.config;
        cfg.timestamp  = getTick();
        logger.log(cfg);

        gui->screenConfig.btnStart.setText("Starting...");

        gui->screenStatus.updateConfig(cfg);

        setupXbee(cfg);
        startTransceiver(cfg);

        // Show status screen
        gui->screenManager.showScreen(XbeeGUI::SCREEN_STATUS);
    }
}

void onStopButtonClick(View* btn, Interaction action)
{
    if (action == Interaction::LONG_CLICK)
    {
        TextView* tvBtn = dynamic_cast<TextView*>(btn);

        if (tvBtn)
        {
            tvBtn->setText("Stopping...");
        }

        if (trans)
        {
            trans->stop();
        }

        if (energyScanner.isRunning())
        {
            energyScanner.stop();
        }

        logger.stop();

        gui->screenManager.showScreen(XbeeGUI::SCREEN_END);
    }
}

void onMarkButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {
        Mark m{getTick(), markCounter++};
        logger.log(m);

        TextView* tvBtn = dynamic_cast<TextView*>(btn);
        if (tvBtn)
        {
            tvBtn->setText("Mark Log (" + to_string(markCounter) + ")");
        }
    }
}

void onEnergyButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {
        energyScanner.start();
        gui->screenManager.showScreen(XbeeGUI::SCREEN_ENERGYSCAN);
    }
}

void startTransceiver(XbeeConfig config)
{
    sndInt.interval = config.sendInterval;
    trans = new XbeeTransceiver(*xbee, logger, sndInt, config.packetSize,
                                config.sendInterval);

    if (!config.txEnabled)
    {
        trans->disableSender();
    }

    trans->start();
}

void setupXbee(XbeeConfig config)
{
    if (config.dataRate80k)
    {
        if (!Xbee::setDataRate(*xbee, true))
        {
            gui->screenStatus.tvCfgDataRate.setBackgroundColor(mxgui::red);
            TRACE("[main] Error setting xbee data rate!\n");
        }
    }

    if (!config.freqHop)
    {
        if (!Xbee::disableFrequencyHopping(*xbee))
        {
            gui->screenStatus.tvCfgFreqHop.setBackgroundColor(mxgui::red);
            TRACE("[main] Error disabling frequency hop!\n");
        }
    }
}

void configure()
{
    // Set SPI pins to correct alternate mode
    GpioSck::mode(Mode::ALTERNATE);
    GpioMiso::mode(Mode::ALTERNATE);
    GpioMosi::mode(Mode::ALTERNATE);

    GpioSck::alternateFunction(5);
    GpioMiso::alternateFunction(5);
    GpioMosi::alternateFunction(5);

    GpioATTN::mode(Mode::INPUT_PULL_UP);

    GpioLedLog::mode(Mode::OUTPUT);
    GpioUserBtn::mode(Mode::INPUT_PULL_DOWN);

    // Set chip select pin to OUTPUT
    GpioCS::mode(Mode::OUTPUT);

    // Chip select starts high (not asserted)
    GpioCS::high();
    GpioLedLog::low();

    // Enable rising-edge interrupt detection on PA2
    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::FALLING_EDGE);
}

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (xbee)
    {
        xbee->handleATTNInterrupt();
    }
}
