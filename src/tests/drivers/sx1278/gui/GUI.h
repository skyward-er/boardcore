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

#pragma once

#include <mxgui/display.h>
#include <radio/SX1278/SX1278.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/ScreenManager.h>
#include <utils/gui/VerticalLayout.h>

std::string format_link_speed(size_t value)
{
    if (value > 1000000)
        return fmt::format("{:.2f} mb/s", static_cast<float>(value) / 1000000);
    else if (value > 1000)
        return fmt::format("{:.2f} kb/s", static_cast<float>(value) / 1000);
    else
        return fmt::format("{} b/s", value);
}

class StatsScreen
{
public:
    struct Data
    {
        uint64_t tx_bitrate;
        uint64_t rx_bitrate;
        int corrupted_count;
        int sent_count;
        int recv_count;
        float packet_loss;
        float rssi;
    };

    StatsScreen()
    {
        status.setAlignment(Boardcore::HorizAlignment::CENTER,
                            Boardcore::VertAlignment::CENTER);
        status.setBackgroundColor(mxgui::red);
        status.setTextColor(mxgui::black);
        status.setFont(mxgui::miscFixedBold);

        lbl_tx_data.setTextColor(mxgui::blue);
        tx_data.setCell(&lbl_tx_bitrate, 0, 0);
        tx_data.setCell(&lbl_sent_count, 1, 0);

        tx_data.setCell(&tx_bitrate, 0, 1);
        tx_data.setCell(&sent_count, 1, 1);

        lbl_rx_data.setTextColor(mxgui::blue);
        rx_data.setCell(&lbl_rx_bitrate, 0, 0);
        rx_data.setCell(&lbl_recv_count, 1, 0);
        rx_data.setCell(&lbl_corrupted_count, 2, 0);
        rx_data.setCell(&lbl_packet_loss, 3, 0);

        rx_data.setCell(&rx_bitrate, 0, 1);
        rx_data.setCell(&recv_count, 1, 1);
        rx_data.setCell(&corrupted_count, 2, 1);
        rx_data.setCell(&packet_loss, 3, 1);

        lbl_misc_data.setTextColor(mxgui::blue);
        misc_data.setCell(&lbl_rssi, 0, 0);
        misc_data.setCell(&rssi, 0, 1);

        root.addView(&status, 0.1);
        root.addView(&lbl_tx_data, 0.1);
        root.addView(&tx_data, 0.4);
        root.addView(&lbl_rx_data, 0.1);
        root.addView(&rx_data, 0.8);
        root.addView(&lbl_misc_data, 0.1);
        root.addView(&misc_data, 0.2);
    }

    void updateError(Boardcore::SX1278::Error value)
    {
        switch (value)
        {
            case Boardcore::SX1278::Error::BAD_VALUE:
                status.setBackgroundColor(mxgui::red);
                status.setText("BAD VALUE");
                break;
            case Boardcore::SX1278::Error::BAD_VERSION:
                status.setBackgroundColor(mxgui::red);
                status.setText("BAD VERSION");
                break;

            default:
                break;
        }
    }

    void updateReady()
    {
        status.setBackgroundColor(mxgui::green);
        status.setText("READY");
    }

    void updateStats(Data &stats)
    {
        tx_bitrate.setText(format_link_speed(stats.tx_bitrate));
        sent_count.setText(fmt::format("{}", stats.sent_count));

        rx_bitrate.setText(format_link_speed(stats.rx_bitrate));
        recv_count.setText(fmt::format("{}", stats.recv_count));
        corrupted_count.setText(fmt::format("{}", stats.corrupted_count));
        packet_loss.setText(
            fmt::format("{:.2f} %", stats.packet_loss * 100.0f));

        rssi.setText(fmt::format("{} dBm", stats.rssi));
    }

    Boardcore::VerticalLayout root{10};
    Boardcore::TextView status{"LOADING"};

    Boardcore::GridLayout tx_data{2, 2};
    Boardcore::GridLayout rx_data{4, 2};
    Boardcore::GridLayout misc_data{1, 2};

    Boardcore::TextView lbl_tx_data{"Tx data"};
    Boardcore::TextView lbl_rx_data{"Rx data"};
    Boardcore::TextView lbl_misc_data{"Misc data"};

    Boardcore::TextView lbl_tx_bitrate{"Tx bitrate:"};
    Boardcore::TextView lbl_sent_count{"Packets sent:"};
    Boardcore::TextView lbl_rx_bitrate{"Rx bitrate:"};
    Boardcore::TextView lbl_recv_count{"Packets received:"};
    Boardcore::TextView lbl_corrupted_count{"Corrupted packets:"};
    Boardcore::TextView lbl_packet_loss{"Packet loss:"};
    Boardcore::TextView lbl_rssi{"RSSI:"};

    Boardcore::TextView tx_bitrate{"0.00 b/s"};
    Boardcore::TextView rx_bitrate{"0.00 b/s"};
    Boardcore::TextView corrupted_count{"0"};
    Boardcore::TextView recv_count{"0"};
    Boardcore::TextView sent_count{"0"};
    Boardcore::TextView packet_loss{"0 %"};
    Boardcore::TextView rssi{"0 dBm"};
};

class GUI
{
public:
    GUI() : screen_manager(mxgui::DisplayManager::instance(), 8)
    {
        screen_manager.addScreen(0, &stats_screen.root);
        screen_manager.start();
    }

    ~GUI() { screen_manager.stop(); }

    Boardcore::ScreenManager screen_manager;
    StatsScreen stats_screen;
};
