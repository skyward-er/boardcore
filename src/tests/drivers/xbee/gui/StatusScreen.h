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

#include <logger/Logger.h>
#include <mxgui/display.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/TextView.h>
#include <utils/gui/VerticalLayout.h>
#include <utils/testutils/ThroughputCalculator.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "../XbeeTestData.h"

using std::to_string;

namespace Boardcore
{

/**
 * @brief Converts tick in milliseconds to the HH:MM:SS format
 */
std::string tickToHMS(long long tick)
{
    char buf[15];

    int h = tick / (1000 * 3600);
    tick -= h * (1000 * 3600);
    int m = tick / (1000 * 60);
    tick -= m * (1000 * 60);
    int s = tick / 1000;

    snprintf(buf, 15, "%02d:%02d:%02d", h, m, s);

    return string(buf);
}

struct StatusScreen
{
    XbeeConfig config;

    StatusScreen()
    {
        title.setFont(mxgui::miscFixedBold);
        title.setTextColor(mxgui::black);
        title.setBackgroundColor(mxgui::green);
        title.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tv_log_status.setFont(mxgui::miscFixedBold);
        tv_log_status.setTextColor(mxgui::white);
        tv_log_status.setBackgroundColor(mxgui::red);
        tv_log_status.setAlignment(HorizAlignment::CENTER,
                                   VertAlignment::CENTER);

        grid_title.setCell(&title, 0, 0);
        grid_title.setCell(&tv_log_status, 0, 1);

        grid_config.setCell(&tv_cfg_txt_tx_enabled, 0, 0);
        grid_config.setCell(&tv_cfg_tx_enabled, 0, 1);

        grid_config.setCell(&tv_cfg_txt_pkt_size, 0, 2);
        grid_config.setCell(&tv_cfg_pkt_size, 0, 3);

        grid_config.setCell(&tv_cfg_txt_snd_interval, 1, 0);
        grid_config.setCell(&tv_cfg_snd_interval, 1, 1);

        grid_config.setCell(&tv_cfg_txt_freq_hop, 1, 2);
        grid_config.setCell(&tv_cfg_freq_hop, 1, 3);

        grid_config.setCell(&tv_cfg_txt_data_rate, 2, 0);
        grid_config.setCell(&tv_cfg_data_rate, 2, 1);

        tv_log_title.setTextColor(mxgui::blue);

        grid_log_status.setCell(&tv_log_title, 0, 0);

        grid_log_status.setCell(&tv_log_txt_buf_written, 1, 0);
        grid_log_status.setCell(&tv_log_buf_written, 1, 1);

        grid_log_status.setCell(&tv_log_txt_buf_ttw, 1, 2);
        grid_log_status.setCell(&tv_log_buf_ttw, 1, 3);

        grid_log_status.setCell(&tv_log_txt_buf_dropped, 2, 0);
        grid_log_status.setCell(&tv_log_buf_dropped, 2, 1);

        grid_log_status.setCell(&tv_log_txt_buf_failed, 2, 2);
        grid_log_status.setCell(&tv_log_buf_failed, 2, 3);

        tv_tx_title.setTextColor(mxgui::blue);

        grid_data.setCell(&tv_tx_title, 0, 0);

        grid_data.setCell(&tv_tx_txt_num_pkt, 1, 0);
        grid_data.setCell(&tv_tx_num_pkt, 1, 1);

        grid_data.setCell(&tv_tx_txt_num_fail, 2, 0);
        grid_data.setCell(&tv_tx_num_fail, 2, 1);

        grid_data.setCell(&tv_tx_txt_pps, 3, 0);
        grid_data.setCell(&tv_tx_pps, 3, 1);

        grid_data.setCell(&tv_tx_txt_TTS, 4, 0);
        grid_data.setCell(&tv_tx_tts, 4, 1);

        grid_data.setCell(&tv_tx_txt_last_status, 5, 0);
        grid_data.setCell(&tv_tx_last_status, 5, 1);

        grid_data.setCell(&tv_tx_txt_last_err, 6, 0);
        grid_data.setCell(&tv_tx_last_err, 6, 1);

        tv_rx_title.setTextColor(mxgui::blue);
        grid_data.setCell(&tv_rx_title, 0, 2);

        grid_data.setCell(&tv_rx_txt_num_pkt, 1, 2);
        grid_data.setCell(&tv_rx_num_pkt, 1, 3);

        grid_data.setCell(&tv_rx_txt_num_fail, 2, 2);
        grid_data.setCell(&tv_rx_num_fail, 2, 3);

        grid_data.setCell(&tv_rx_txt_lost, 3, 2);
        grid_data.setCell(&tv_rx_lost, 3, 3);

        grid_data.setCell(&tv_rx_txt_RSSI, 4, 2);
        grid_data.setCell(&tv_rx_RSSI, 4, 3);

        grid_data.setCell(&tv_rx_txt_data_rate, 5, 2);
        grid_data.setCell(&tv_rx_data_rate, 5, 3);

        grid_data.setCell(&tv_rx_txt_pps, 6, 2);
        grid_data.setCell(&tv_rx_pps, 6, 3);

        grid_data.setCell(&tv_rx_txt_time_since_last_rx, 7, 2);
        grid_data.setCell(&tv_rx_time_since_last_rx, 7, 3);

        btn_mark.setSelectable(true);
        btn_mark.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_mark.setBackgroundColor(mxgui::darkGrey);

        btn_stop.setSelectable(true);
        btn_stop.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_stop.setBackgroundColor(mxgui::darkGrey);

        grid_buttons.setCell(&btn_mark, 0);
        grid_buttons.setCell(&btn_stop, 1);
        grid_buttons.setDrawBorder(true);

        root.addView(&grid_title, 0.8);
        root.addView(&grid_config, 1.5);
        root.addView(&grid_log_status, 1.5);
        root.addView(&grid_data, 5);
        root.addView(&grid_buttons, 1);
    }

    void updateConfig(XbeeConfig cfg)
    {
        // Update GUI with selected config values
        tv_cfg_tx_enabled.setText(cfg.tx_enabled ? "Enabled" : "Disabled");
        tv_cfg_pkt_size.setText(std::to_string(cfg.packet_size));
        tv_cfg_snd_interval.setText(cfg.send_interval == 0
                                        ? "Cont"
                                        : std::to_string(cfg.send_interval));

        tv_cfg_freq_hop.setText(cfg.freq_hop ? "Enabled" : "Disabled");
        tv_cfg_data_rate.setText(cfg.data_rate_80k ? "80 kbps" : "10 kbps");
    }

    void updateLogStatus(Logger& logger)
    {
        LogStats stats = logger.getLogStats();

        if (logger.getLogNumber() >= 0)
        {
            string log_name = logger.getFileName(logger.getLogNumber());

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

        tv_log_buf_dropped.setText(to_string(stats.statDroppedSamples));

        if (stats.statDroppedSamples > 0)
        {
            tv_log_buf_dropped.setBackgroundColor(mxgui::red);
        }

        tv_log_buf_failed.setText(to_string(stats.statWriteFailed) + "   (" +
                                  to_string(stats.statWriteError) + ")");

        if (stats.statWriteError != 0)
        {
            tv_log_buf_failed.setBackgroundColor(mxgui::red);
        }

        tv_log_buf_written.setText(to_string(stats.statBufferWritten));
        tv_log_buf_ttw.setText(to_string(stats.statWriteTime) + " ms");
    }

    void updateXbeeStatus(DataRateResult res_rcv, DataRateResult res_snd,
                          TxData txd, RxData rxd, Xbee::XbeeStatus xbee_status)
    {
        char str_buf[30];

        tv_tx_num_pkt.setText(
            to_string(txd.tx_success_counter + txd.tx_fail_counter));
        tv_tx_num_fail.setText(to_string(txd.tx_fail_counter));

        snprintf(str_buf, 30, "%.1f pkt/s", res_snd.packets_per_second);

        tv_tx_pps.setText(str_buf);
        tv_tx_tts.setText(to_string(txd.time_to_send) + " ms");

        tv_tx_last_status.setText(to_string(xbee_status.last_tx_status));
        tv_tx_last_err.setText(to_string(xbee_status.last_tx_status_error));

        tv_rx_num_pkt.setText(to_string(rxd.rcv_count));
        tv_rx_num_fail.setText(to_string(rxd.rcv_errors));
        // tv_rx_num_fail.setText(to_string(int_counter) + " " +
        //                               to_string(GpioATTN::value()));
        tv_rx_lost.setText(to_string(rxd.packets_lost));

        tv_rx_RSSI.setText(to_string(rxd.RSSI) + " dB");

        snprintf(str_buf, 30, "%.0f B/s", res_rcv.data_rate);
        tv_rx_data_rate.setText(str_buf);

        snprintf(str_buf, 30, "%.1f pkt/s", res_rcv.packets_per_second);
        tv_rx_pps.setText(str_buf);

        tv_rx_time_since_last_rx.setText(
            tickToHMS(miosix::getTick() - rxd.last_packet_timestamp));
    }

    VerticalLayout root{10};

    TextView tv_cfg_tx_enabled{"Disabled"};
    TextView tv_cfg_pkt_size{"256 B"};
    TextView tv_cfg_snd_interval{"Cont"};
    TextView tv_cfg_freq_hop{"Enabled"};
    TextView tv_cfg_data_rate{"10 kbps"};

    TextView tv_log_status{"SD ERR"};

    TextView tv_log_buf_dropped{"0"};
    TextView tv_log_buf_failed{"0 (0)"};
    TextView tv_log_buf_written{"0"};
    TextView tv_log_buf_ttw{"0 ms"};

    TextView tv_tx_num_pkt{"0"};
    TextView tv_tx_num_fail{"0"};
    TextView tv_tx_pps{"0 pkt/s"};
    TextView tv_tx_tts{"- ms"};
    TextView tv_tx_last_status{"0"};
    TextView tv_tx_last_err{"0"};

    TextView tv_rx_num_pkt{"0"};
    TextView tv_rx_num_fail{"0"};
    TextView tv_rx_lost{"0"};
    TextView tv_rx_RSSI{"-40 dB"};
    TextView tv_rx_data_rate{"0 B/s"};
    TextView tv_rx_pps{"0 pkt/s"};
    TextView tv_rx_time_since_last_rx{"00:00:00"};

    TextView btn_mark{"Mark Log (1)"};
    TextView btn_stop{"Stop"};

private:
    TextView title{"Xbee Status"};
    GridLayout grid_title{1, 2};
    GridLayout grid_config{3, 4};
    GridLayout grid_buttons{1, 2};
    GridLayout grid_log_status{3, 4};
    GridLayout grid_data{8, 4};

    TextView tv_cfg_txt_tx_enabled{"TX"};
    TextView tv_cfg_txt_pkt_size{"Pkt size"};
    TextView tv_cfg_txt_snd_interval{"Interv"};
    TextView tv_cfg_txt_freq_hop{"Freq hop"};
    TextView tv_cfg_txt_data_rate{"Data rate"};

    TextView tv_log_title{"LOG"};
    TextView tv_log_txt_buf_dropped{"Buf drops"};
    TextView tv_log_txt_buf_failed{"Wrt fails"};
    TextView tv_log_txt_buf_written{"Wrt succ"};
    TextView tv_log_txt_buf_ttw{"TTW"};

    TextView tv_tx_title{"TX"};

    TextView tv_tx_txt_num_pkt{"Sent"};
    TextView tv_tx_txt_num_fail{"Fails"};
    TextView tv_tx_txt_pps{"PPS"};
    TextView tv_tx_txt_TTS{"TTS"};
    TextView tv_tx_txt_last_status{"Status"};
    TextView tv_tx_txt_last_err{"Last err"};

    TextView tv_rx_title{"RX"};

    TextView tv_rx_txt_num_pkt{"Recv"};
    TextView tv_rx_txt_num_fail{"Fails"};
    TextView tv_rx_txt_lost{"Lost"};
    TextView tv_rx_txt_RSSI{"RSSI"};
    TextView tv_rx_txt_data_rate{"DR"};
    TextView tv_rx_txt_pps{"PPS"};
    TextView tv_rx_txt_time_since_last_rx{"No RX dt"};
};

}  // namespace Boardcore
