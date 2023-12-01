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
#include <utils/TestUtils/ThroughputCalculator.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/TextView.h>
#include <utils/gui/VerticalLayout.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "../XbeeTestData.h"

using std::to_string;

namespace Boardcore
{

/**
 * @brief Converts nanoseconds to the HH:MM:SS format
 */
std::string nanosToHMS(long long nanos)
{
    long long millis = nanos / Constants::NS_IN_MS;
    char buf[20];

    lldiv_t hours   = std::div(millis, 3600000ll);
    long long h     = hours.quot;
    lldiv_t minutes = std::div(hours.rem, 60000ll);
    long long m     = minutes.quot;
    long long s     = (minutes.rem / 1000);

    snprintf(buf, 20, "%02lld:%02lld:%02lld", h, m, s);

    return string(buf);
}

struct StatusScreen
{
    XbeeConfig config = {};

    StatusScreen()
    {
        title.setFont(mxgui::miscFixedBold);
        title.setTextColor(mxgui::black);
        title.setBackgroundColor(mxgui::green);
        title.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tvLogStatus.setFont(mxgui::miscFixedBold);
        tvLogStatus.setTextColor(mxgui::white);
        tvLogStatus.setBackgroundColor(mxgui::red);
        tvLogStatus.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        gridTitle.setCell(&title, 0, 0);
        gridTitle.setCell(&tvLogStatus, 0, 1);

        gridConfig.setCell(&tvCfgTxtTxEnabled, 0, 0);
        gridConfig.setCell(&tvCfgTxEnabled, 0, 1);

        gridConfig.setCell(&tvCfgTxtPktSize, 0, 2);
        gridConfig.setCell(&tvCfgPktSize, 0, 3);

        gridConfig.setCell(&tvCfgTxtSndInterval, 1, 0);
        gridConfig.setCell(&tvCfgSndInterval, 1, 1);

        gridConfig.setCell(&tvCfgTxtFreqHop, 1, 2);
        gridConfig.setCell(&tvCfgFreqHop, 1, 3);

        gridConfig.setCell(&tvCfgTxtDataRate, 2, 0);
        gridConfig.setCell(&tvCfgDataRate, 2, 1);

        tvLogTitle.setTextColor(mxgui::blue);

        gridLogStatus.setCell(&tvLogTitle, 0, 0);

        gridLogStatus.setCell(&tvLogTxtBufWritten, 1, 0);
        gridLogStatus.setCell(&tvLogBufWritten, 1, 1);

        gridLogStatus.setCell(&tvLogTxtBufTtw, 1, 2);
        gridLogStatus.setCell(&tvLogBufTtw, 1, 3);

        gridLogStatus.setCell(&tvLogTxtBufDropped, 2, 0);
        gridLogStatus.setCell(&tvLogBufDropped, 2, 1);

        gridLogStatus.setCell(&tvLogTxtBufFailed, 2, 2);
        gridLogStatus.setCell(&tvLogBufFailed, 2, 3);

        tvTxTitle.setTextColor(mxgui::blue);

        gridData.setCell(&tvTxTitle, 0, 0);

        gridData.setCell(&tvTxTxtNumPkt, 1, 0);
        gridData.setCell(&tvTxNumPkt, 1, 1);

        gridData.setCell(&tvTxTxtNumFail, 2, 0);
        gridData.setCell(&tvTxNumFail, 2, 1);

        gridData.setCell(&tvTxTxtPps, 3, 0);
        gridData.setCell(&tvTxPps, 3, 1);

        gridData.setCell(&tvTxTxtTTS, 4, 0);
        gridData.setCell(&tvTxTts, 4, 1);

        gridData.setCell(&tvTxTxtLastStatus, 5, 0);
        gridData.setCell(&tvTxLastStatus, 5, 1);

        gridData.setCell(&tvTxTxtLastErr, 6, 0);
        gridData.setCell(&tvTxLastErr, 6, 1);

        tvRxTitle.setTextColor(mxgui::blue);
        gridData.setCell(&tvRxTitle, 0, 2);

        gridData.setCell(&tvRxTxtNumPkt, 1, 2);
        gridData.setCell(&tvRxNumPkt, 1, 3);

        gridData.setCell(&tvRxTxtNumFail, 2, 2);
        gridData.setCell(&tvRxNumFail, 2, 3);

        gridData.setCell(&tvRxTxtLost, 3, 2);
        gridData.setCell(&tvRxLost, 3, 3);

        gridData.setCell(&tvRxTxtRSSI, 4, 2);
        gridData.setCell(&tvRx_RSSI, 4, 3);

        gridData.setCell(&tvRxTxtDataRate, 5, 2);
        gridData.setCell(&tvRxDataRate, 5, 3);

        gridData.setCell(&tvRxTxtPps, 6, 2);
        gridData.setCell(&tvRxPps, 6, 3);

        gridData.setCell(&tvRxTxtTimeSinceLastRx, 7, 2);
        gridData.setCell(&tvRxTimeSinceLastRx, 7, 3);

        btnMark.setSelectable(true);
        btnMark.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnMark.setBackgroundColor(mxgui::darkGrey);

        btnStop.setSelectable(true);
        btnStop.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnStop.setBackgroundColor(mxgui::darkGrey);

        gridButtons.setCell(&btnMark, 0);
        gridButtons.setCell(&btnStop, 1);
        gridButtons.setDrawBorder(true);

        root.addView(&gridTitle, 0.8);
        root.addView(&gridConfig, 1.5);
        root.addView(&gridLogStatus, 1.5);
        root.addView(&gridData, 5);
        root.addView(&gridButtons, 1);
    }

    void updateConfig(XbeeConfig cfg)
    {
        // Update GUI with selected config values
        tvCfgTxEnabled.setText(cfg.txEnabled ? "Enabled" : "Disabled");
        tvCfgPktSize.setText(std::to_string(cfg.packetSize));
        tvCfgSndInterval.setText(
            cfg.sendInterval == 0 ? "Cont" : std::to_string(cfg.sendInterval));

        tvCfgFreqHop.setText(cfg.freqHop ? "Enabled" : "Disabled");
        tvCfgDataRate.setText(cfg.dataRate80k ? "80 kbps" : "10 kbps");
    }

    void updateLogStatus(Logger& logger)
    {
        LoggerStats stats = logger.getStats();

        if (logger.getCurrentLogNumber() >= 0)
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

        tvLogBufDropped.setText(to_string(stats.droppedSamples));

        if (stats.droppedSamples > 0)
        {
            tvLogBufDropped.setBackgroundColor(mxgui::red);
        }

        tvLogBufFailed.setText(to_string(stats.writesFailed) + "   (" +
                               to_string(stats.lastWriteError) + ")");

        if (stats.lastWriteError != 0)
        {
            tvLogBufFailed.setBackgroundColor(mxgui::red);
        }

        tvLogBufWritten.setText(to_string(stats.buffersWritten));
        tvLogBufTtw.setText(to_string(stats.averageWriteTime) + " ms");
    }

    void updateXbeeStatus(DataRateResult resRcv, DataRateResult resSnd,
                          TxData txd, RxData rxd, Xbee::XbeeStatus xbeeStatus)
    {
        char strBuf[30];

        tvTxNumPkt.setText(to_string(txd.txSuccessCounter + txd.txFailCounter));
        tvTxNumFail.setText(to_string(txd.txFailCounter));

        snprintf(strBuf, 30, "%.1f pkt/s", resSnd.packetsPerSecond);

        tvTxPps.setText(strBuf);
        tvTxTts.setText(to_string(txd.timeToSend) + " ms");

        tvTxLastStatus.setText(to_string(xbeeStatus.lastTxStatus));
        tvTxLastErr.setText(to_string(xbeeStatus.lastTxStatusError));

        tvRxNumPkt.setText(to_string(rxd.rcvCount));
        tvRxNumFail.setText(to_string(rxd.rcvErrors));
        // tvRxNumFail.setText(to_string(int_counter) + " " +
        //                               to_string(GpioATTN::value()));
        tvRxLost.setText(to_string(rxd.packetsLost));

        tvRx_RSSI.setText(to_string(rxd.RSSI) + " dB");

        snprintf(strBuf, 30, "%.0f B/s", resRcv.dataRate);
        tvRxDataRate.setText(strBuf);

        snprintf(strBuf, 30, "%.1f pkt/s", resRcv.packetsPerSecond);
        tvRxPps.setText(strBuf);

        tvRxTimeSinceLastRx.setText(
            nanosToHMS(miosix::getTime() - rxd.lastPacketTimestamp));
    }

    VerticalLayout root{10};

    TextView tvCfgTxEnabled{"Disabled"};
    TextView tvCfgPktSize{"256 B"};
    TextView tvCfgSndInterval{"Cont"};
    TextView tvCfgFreqHop{"Enabled"};
    TextView tvCfgDataRate{"10 kbps"};

    TextView tvLogStatus{"SD ERR"};

    TextView tvLogBufDropped{"0"};
    TextView tvLogBufFailed{"0 (0)"};
    TextView tvLogBufWritten{"0"};
    TextView tvLogBufTtw{"0 ms"};

    TextView tvTxNumPkt{"0"};
    TextView tvTxNumFail{"0"};
    TextView tvTxPps{"0 pkt/s"};
    TextView tvTxTts{"- ms"};
    TextView tvTxLastStatus{"0"};
    TextView tvTxLastErr{"0"};

    TextView tvRxNumPkt{"0"};
    TextView tvRxNumFail{"0"};
    TextView tvRxLost{"0"};
    TextView tvRx_RSSI{"-40 dB"};
    TextView tvRxDataRate{"0 B/s"};
    TextView tvRxPps{"0 pkt/s"};
    TextView tvRxTimeSinceLastRx{"00:00:00"};

    TextView btnMark{"Mark Log (1)"};
    TextView btnStop{"Stop"};

private:
    TextView title{"Xbee Status"};
    GridLayout gridTitle{1, 2};
    GridLayout gridConfig{3, 4};
    GridLayout gridButtons{1, 2};
    GridLayout gridLogStatus{3, 4};
    GridLayout gridData{8, 4};

    TextView tvCfgTxtTxEnabled{"TX"};
    TextView tvCfgTxtPktSize{"Pkt size"};
    TextView tvCfgTxtSndInterval{"Interv"};
    TextView tvCfgTxtFreqHop{"Freq hop"};
    TextView tvCfgTxtDataRate{"Data rate"};

    TextView tvLogTitle{"LOG"};
    TextView tvLogTxtBufDropped{"Buf drops"};
    TextView tvLogTxtBufFailed{"Wrt fails"};
    TextView tvLogTxtBufWritten{"Wrt succ"};
    TextView tvLogTxtBufTtw{"TTW"};

    TextView tvTxTitle{"TX"};

    TextView tvTxTxtNumPkt{"Sent"};
    TextView tvTxTxtNumFail{"Fails"};
    TextView tvTxTxtPps{"PPS"};
    TextView tvTxTxtTTS{"TTS"};
    TextView tvTxTxtLastStatus{"Status"};
    TextView tvTxTxtLastErr{"Last err"};

    TextView tvRxTitle{"RX"};

    TextView tvRxTxtNumPkt{"Recv"};
    TextView tvRxTxtNumFail{"Fails"};
    TextView tvRxTxtLost{"Lost"};
    TextView tvRxTxtRSSI{"RSSI"};
    TextView tvRxTxtDataRate{"DR"};
    TextView tvRxTxtPps{"PPS"};
    TextView tvRxTxtTimeSinceLastRx{"No RX dt"};
};

}  // namespace Boardcore
