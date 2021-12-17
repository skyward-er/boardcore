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
Logger& logger   = Logger::instance();
Xbee::Xbee* xbee = nullptr;
ConstSendInterval snd_int{0};
XbeeTransceiver* trans = nullptr;
XbeeGUI* gui;
ButtonHandler<GpioUserBtn>* btn_handler;

unsigned int mark_counter = 1;

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

                gui->screen_energy.updateScan(scan);

                EnergyScanData data{getTick(), scan};
                logger.log(data);
            }
        }
    }
} energy_scanner;

int main()
{
    // Hardware
    configure();

    // SD
    try
    {
        logger.start();
        printf("\nLog file opened! (%s)\n\n",
               logger.getFileName(logger.getLogNumber()).c_str());
    }
    catch (const std::runtime_error& err)
    {
        GpioLedLog::high();
        printf("\n!!!!!!Error opening log file!!!!!!!\n\n");
    }

    // XBee
    SPIBus spi_bus(SPI1);
    SPIBusConfig cfg{};
    cfg.clockDivider = SPI::ClockDivider::DIV_16;

    GpioPin cs   = GpioCS::getPin();
    GpioPin attn = GpioATTN::getPin();
    GpioPin rst  = GpioRST::getPin();

    xbee = new Xbee::Xbee(spi_bus, cfg, cs, attn, rst);

    // GUI
    gui = new XbeeGUI();

    btn_handler = new ButtonHandler<GpioUserBtn>(
        0, bind(&ScreenManager::onButtonPress, &gui->screen_manager, _2));

    btn_handler->start();

    gui->screen_config.btn_start.addOnInteractionListener(onStartButtonClick);
    gui->screen_config.btn_energy.addOnInteractionListener(onEnergyButtonClick);

    gui->screen_status.btn_stop.addOnInteractionListener(onStopButtonClick);
    gui->screen_status.btn_mark.addOnInteractionListener(onMarkButtonClick);

    gui->screen_energy.btn_stop.addOnInteractionListener(onStopButtonClick);
    gui->screen_energy.btn_mark.addOnInteractionListener(onMarkButtonClick);
    gui->screen_energy.btn_reset.addOnInteractionListener(
        [&](View* d, Interaction action)
        {
            UNUSED(d);
            if (action == Interaction::CLICK)
                gui->screen_energy.resetStats();
        });

    gui->screen_end.tv_f.addOnInteractionListener(
        [&](View* d, Interaction action)
        {
            UNUSED(d);
            if (action == Interaction::CLICK)
                gui->screen_manager.showScreen(XbeeGUI::SCREEN_RESPECT);
        });

    gui->screen_end.tv_reset.addOnInteractionListener(
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
        switch (gui->screen_manager.getScreen())
        {
            case XbeeGUI::SCREEN_CONFIG:
                gui->screen_config.updateLogStatus(logger);
                break;
            case XbeeGUI::SCREEN_STATUS:
                if (trans && xbee)
                {
                    gui->screen_status.updateXbeeStatus(
                        trans->getReceiver().getDataRate(),
                        trans->getSender().getDataRate(),
                        trans->getSender().getTxData(),
                        trans->getReceiver().getRxData(), xbee->getStatus());

                    logger.log(xbee->getStatus());
                }

                gui->screen_status.updateLogStatus(logger);
                break;
            case XbeeGUI::SCREEN_ENERGYSCAN:
                gui->screen_energy.updateLogStatus(logger);
                break;
            default:
                break;
        }

        logger.log(logger.getLogStats());
        Thread::sleepUntil(start + 500);
    }
}

void onStartButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {

        XbeeConfig cfg = gui->screen_config.config;
        cfg.timestamp  = getTick();
        logger.log(cfg);

        gui->screen_config.btn_start.setText("Starting...");

        gui->screen_status.updateConfig(cfg);

        setupXbee(cfg);
        startTransceiver(cfg);

        // Show status screen
        gui->screen_manager.showScreen(XbeeGUI::SCREEN_STATUS);
    }
}

void onStopButtonClick(View* btn, Interaction action)
{
    if (action == Interaction::LONG_CLICK)
    {
        TextView* tv_btn = dynamic_cast<TextView*>(btn);

        if (tv_btn)
        {
            tv_btn->setText("Stopping...");
        }

        if (trans)
        {
            trans->stop();
        }

        if (energy_scanner.isRunning())
        {
            energy_scanner.stop();
        }

        logger.stop();

        gui->screen_manager.showScreen(XbeeGUI::SCREEN_END);
    }
}

void onMarkButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {
        Mark m{getTick(), mark_counter++};
        logger.log(m);

        TextView* tv_btn = dynamic_cast<TextView*>(btn);
        if (tv_btn)
        {
            tv_btn->setText("Mark Log (" + to_string(mark_counter) + ")");
        }
    }
}

void onEnergyButtonClick(View* btn, Interaction action)
{
    UNUSED(btn);
    if (action == Interaction::CLICK)
    {
        energy_scanner.start();
        gui->screen_manager.showScreen(XbeeGUI::SCREEN_ENERGYSCAN);
    }
}

void startTransceiver(XbeeConfig config)
{
    snd_int.interval = config.send_interval;
    trans = new XbeeTransceiver(*xbee, logger, snd_int, config.packet_size,
                                config.send_interval);

    if (!config.tx_enabled)
    {
        trans->disableSender();
    }

    trans->start();
}

void setupXbee(XbeeConfig config)
{
    if (config.data_rate_80k)
    {
        if (!Xbee::setDataRate(*xbee, true))
        {
            gui->screen_status.tv_cfg_data_rate.setBackgroundColor(mxgui::red);
            TRACE("[main] Error setting xbee data rate!\n");
        }
    }

    if (!config.freq_hop)
    {
        if (!Xbee::disableFrequencyHopping(*xbee))
        {
            gui->screen_status.tv_cfg_freq_hop.setBackgroundColor(mxgui::red);
            TRACE("[main] Error disabling frequency hop!\n");
        }
    }
}

void configure()
{
    {
        FastInterruptDisableLock dLock;

        // Enable SPI5 and TIM5 peripheral clocks
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

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
    }

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
