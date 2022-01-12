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

#pragma once

#include <mxgui/display.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/TextView.h>
#include <utils/gui/VerticalLayout.h>

#include <cstdint>

#include "../XbeeTestData.h"

namespace Boardcore
{

struct ConfigScreen
{
    XbeeConfig config;

    enum OptSendEnableIDs : uint8_t
    {
        TX_ENABLED,
        TX_DISABLED
    };

    enum OptFreqHopIDs : uint8_t
    {
        FH_ENABLED,
        FH_DISABLED
    };

    enum OptDataRateIDs : uint8_t
    {
        DR_10K,
        DR_80K
    };

    enum OptPacketSizeIDs : uint8_t
    {
        PKTSIZE_64,
        PKTSIZE_128,
        PKTSIZE_256
    };

    enum OptSendIntervalIDs : uint8_t
    {
        TXINT_CONTINUOUS,
        TXINT_200MS,
        TXINT_250MS,
        TXINT_333MS,
        TXINT_500MS,
        TXINT_1000MS
    };

    ConfigScreen()
    {
        title.setFont(mxgui::miscFixedBold);
        title.setTextColor(mxgui::black);
        title.setBackgroundColor(mxgui::green);
        title.setAlignment(HorizAlignment::LEFT, VertAlignment::CENTER);

        tvLogStatus.setFont(mxgui::miscFixedBold);
        tvLogStatus.setTextColor(mxgui::white);
        tvLogStatus.setBackgroundColor(mxgui::red);
        tvLogStatus.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        gridTitle.setCell(&title, 0, 0);
        gridTitle.setCell(&tvLogStatus, 0, 1);

        grid.setCell(&btnEnergy, 0);
        grid.setCell(&btnStart, 1);
        grid.setDrawBorder(true);

        btnStart.setSelectable(true);
        btnStart.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnStart.setBackgroundColor(mxgui::darkGrey);

        btnEnergy.setSelectable(true);
        btnEnergy.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnEnergy.setBackgroundColor(mxgui::darkGrey);

        root.addView(&gridTitle, 0.6);
        root.addView(&optSendEnable, 1);
        root.addView(&optPacketSize, 1);

        root.addView(&optSendInterval, 2);

        root.addView(&optFreqHop, 1);
        root.addView(&optDataRate, 1);
        root.addView(&grid, 1);

        addOptionCallbacks();
    }

    void updateLogStatus(Logger& logger)
    {
        if (logger.getLogNumber() >= 0)
        {
            string logName = logger.getCurrentFileName();

            tvLogStatus.setText(logName);
            tvLogStatus.setTextColor(mxgui::black);
            tvLogStatus.setBackgroundColor(mxgui::green);
        }
        else
        {
            tvLogStatus.setText("SD ERR");
            tvLogStatus.setTextColor(mxgui::white);
            tvLogStatus.setBackgroundColor(mxgui::red);
        }
    }

    VerticalLayout root{10};

    TextView btnStart{"Start"};
    TextView btnEnergy{"Energy scan"};

private:
    void addOptionCallbacks()
    {
        optSendEnable.addOnOptionChosenListener(
            [&](unsigned int id) { config.txEnabled = id == TX_ENABLED; });

        optSendInterval.addOnOptionChosenListener(
            [&](unsigned int id)
            {
                switch (id)
                {
                    case TXINT_CONTINUOUS:
                        config.sendInterval = 0;
                        break;
                    case TXINT_200MS:
                        config.sendInterval = 200;
                        break;
                    case TXINT_250MS:
                        config.sendInterval = 250;
                        break;
                    case TXINT_333MS:
                        config.sendInterval = 333;
                        break;
                    case TXINT_500MS:
                        config.sendInterval = 500;
                        break;
                    case TXINT_1000MS:
                        config.sendInterval = 1000;
                        break;
                }
            });

        optPacketSize.addOnOptionChosenListener(
            [&](unsigned int id)
            {
                switch (id)
                {
                    case PKTSIZE_64:
                        config.packetSize = 64;
                        break;
                    case PKTSIZE_128:
                        config.packetSize = 128;
                        break;
                    case PKTSIZE_256:
                        config.packetSize = 256;
                        break;
                }
            });

        optFreqHop.addOnOptionChosenListener(
            [&](unsigned int id) { config.freqHop = id == FH_ENABLED; });

        optDataRate.addOnOptionChosenListener(
            [&](unsigned int id) { config.dataRate80k = id == DR_80K; });
    }

    OptionView optSendEnable{
        "Transmission",
        {{TX_ENABLED, "Enabled"}, {TX_DISABLED, "Disabled"}},
        TX_DISABLED,
        2};

    OptionView optPacketSize{
        "TX Packet Size",
        {{PKTSIZE_64, "64 B"}, {PKTSIZE_128, "128 B"}, {PKTSIZE_256, "256 B"}},
        PKTSIZE_256,
        3};

    OptionView optSendInterval{"Send Interval",
                               {{TXINT_CONTINUOUS, "Cont"},
                                {TXINT_200MS, "200 ms"},
                                {TXINT_250MS, "250 ms"},
                                {TXINT_333MS, "333 ms"},
                                {TXINT_500MS, "500 ms"},
                                {TXINT_1000MS, "1 sec"}},
                               TXINT_CONTINUOUS,
                               4};

    OptionView optFreqHop{"Frequency hopping",
                          {{FH_ENABLED, "Enabled"}, {FH_DISABLED, "Disabled"}},
                          FH_ENABLED,
                          2};
    OptionView optDataRate{
        "Xbee datarate", {{DR_10K, "10 kbps"}, {DR_80K, "80 kbps"}}, DR_10K, 2};

    GridLayout grid{1, 2};
    GridLayout gridTitle{1, 2};
    TextView title{"Xbee Setup"};
    TextView tvLogStatus{"SD ERR!"};
};

}  // namespace Boardcore
