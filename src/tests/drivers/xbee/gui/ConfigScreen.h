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

        tv_log_status.setFont(mxgui::miscFixedBold);
        tv_log_status.setTextColor(mxgui::white);
        tv_log_status.setBackgroundColor(mxgui::red);
        tv_log_status.setAlignment(HorizAlignment::CENTER,
                                   VertAlignment::CENTER);

        grid_title.setCell(&title, 0, 0);
        grid_title.setCell(&tv_log_status, 0, 1);

        grid.setCell(&btn_energy, 0);
        grid.setCell(&btn_start, 1);
        grid.setDrawBorder(true);

        btn_start.setSelectable(true);
        btn_start.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_start.setBackgroundColor(mxgui::darkGrey);

        btn_energy.setSelectable(true);
        btn_energy.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_energy.setBackgroundColor(mxgui::darkGrey);

        root.addView(&grid_title, 0.6);
        root.addView(&opt_send_enable, 1);
        root.addView(&opt_packet_size, 1);

        root.addView(&opt_send_interval, 2);

        root.addView(&opt_freq_hop, 1);
        root.addView(&opt_data_rate, 1);
        root.addView(&grid, 1);

        addOptionCallbacks();
    }

    void updateLogStatus(Logger& logger)
    {
        if (logger.getLogNumber() >= 0)
        {
            string log_name = logger.getCurrentFileName();

            tv_log_status.setText(log_name);
            tv_log_status.setTextColor(mxgui::black);
            tv_log_status.setBackgroundColor(mxgui::green);
        }
        else
        {
            tv_log_status.setText("SD ERR");
            tv_log_status.setTextColor(mxgui::white);
            tv_log_status.setBackgroundColor(mxgui::red);
        }
    }

    VerticalLayout root{10};

    TextView btn_start{"Start"};
    TextView btn_energy{"Energy scan"};

private:
    void addOptionCallbacks()
    {
        opt_send_enable.addOnOptionChosenListener(
            [&](unsigned int id) { config.tx_enabled = id == TX_ENABLED; });

        opt_send_interval.addOnOptionChosenListener(
            [&](unsigned int id)
            {
                switch (id)
                {
                    case TXINT_CONTINUOUS:
                        config.send_interval = 0;
                        break;
                    case TXINT_200MS:
                        config.send_interval = 200;
                        break;
                    case TXINT_250MS:
                        config.send_interval = 250;
                        break;
                    case TXINT_333MS:
                        config.send_interval = 333;
                        break;
                    case TXINT_500MS:
                        config.send_interval = 500;
                        break;
                    case TXINT_1000MS:
                        config.send_interval = 1000;
                        break;
                }
            });

        opt_packet_size.addOnOptionChosenListener(
            [&](unsigned int id)
            {
                switch (id)
                {
                    case PKTSIZE_64:
                        config.packet_size = 64;
                        break;
                    case PKTSIZE_128:
                        config.packet_size = 128;
                        break;
                    case PKTSIZE_256:
                        config.packet_size = 256;
                        break;
                }
            });

        opt_freq_hop.addOnOptionChosenListener(
            [&](unsigned int id) { config.freq_hop = id == FH_ENABLED; });

        opt_data_rate.addOnOptionChosenListener(
            [&](unsigned int id) { config.data_rate_80k = id == DR_80K; });
    }

    OptionView opt_send_enable{
        "Transmission",
        {{TX_ENABLED, "Enabled"}, {TX_DISABLED, "Disabled"}},
        TX_DISABLED,
        2};

    OptionView opt_packet_size{
        "TX Packet Size",
        {{PKTSIZE_64, "64 B"}, {PKTSIZE_128, "128 B"}, {PKTSIZE_256, "256 B"}},
        PKTSIZE_256,
        3};

    OptionView opt_send_interval{"Send Interval",
                                 {{TXINT_CONTINUOUS, "Cont"},
                                  {TXINT_200MS, "200 ms"},
                                  {TXINT_250MS, "250 ms"},
                                  {TXINT_333MS, "333 ms"},
                                  {TXINT_500MS, "500 ms"},
                                  {TXINT_1000MS, "1 sec"}},
                                 TXINT_CONTINUOUS,
                                 4};

    OptionView opt_freq_hop{
        "Frequency hopping",
        {{FH_ENABLED, "Enabled"}, {FH_DISABLED, "Disabled"}},
        FH_ENABLED,
        2};
    OptionView opt_data_rate{
        "Xbee datarate", {{DR_10K, "10 kbps"}, {DR_80K, "80 kbps"}}, DR_10K, 2};

    GridLayout grid{1, 2};
    GridLayout grid_title{1, 2};
    TextView title{"Xbee Setup"};
    TextView tv_log_status{"SD ERR!"};
};

}  // namespace Boardcore
