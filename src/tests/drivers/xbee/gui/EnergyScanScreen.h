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

#include <math/Stats.h>
#include <mxgui/display.h>
#include <utils/gui/GridLayout.h>
#include <utils/gui/TextView.h>
#include <utils/gui/VerticalLayout.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>

using std::array;
using std::to_string;

namespace Boardcore
{

struct EnergyScanScreen
{
    static constexpr unsigned int NUM_CHANNELS     = 30;
    static constexpr unsigned int NUM_CHANNEL_ROWS = (NUM_CHANNELS + 1) / 2;
    static constexpr unsigned int NUM_COLS         = 8;

    static constexpr float COLOR_PERCENTILE = 0.15f;
    EnergyScanScreen()
    {
        tvTitle.setFont(mxgui::miscFixedBold);
        tvTitle.setTextColor(mxgui::black);
        tvTitle.setBackgroundColor(mxgui::green);
        tvTitle.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tvLogStatus.setFont(mxgui::miscFixedBold);
        tvLogStatus.setTextColor(mxgui::white);
        tvLogStatus.setBackgroundColor(mxgui::red);
        tvLogStatus.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        gridTitle.setCell(&tvTitle, 0, 0);
        gridTitle.setCell(&tvLogStatus, 0, 1);

        for (unsigned int i = 0; i < NUM_COLS; i++)
        {
            colTitles[i] = new TextView(titles[i % (NUM_COLS / 2)]);
            colTitles[i]->setTextColor(mxgui::blue);

            gridChannels.setCell(colTitles[i], 0, i);
        }

        mxgui::Color pink(0xFC59);
        for (unsigned int i = 0; i < NUM_CHANNELS; ++i)
        {
            char buf[3];
            snprintf(buf, 3, "%02u", i);

            colNames[i] = new TextView(buf);
            colNames[i]->setTextColor(pink);

            colCurrent[i] = new TextView("-40");
            colMin[i]     = new TextView("-40");
            colMax[i]     = new TextView("-40");

            gridChannels.setCell(colNames[i], (i % NUM_CHANNEL_ROWS) + 1,
                                 (i / NUM_CHANNEL_ROWS) * 4 + 0);
            gridChannels.setCell(colCurrent[i], (i % NUM_CHANNEL_ROWS) + 1,
                                 (i / NUM_CHANNEL_ROWS) * 4 + 1);
            gridChannels.setCell(colMin[i], (i % NUM_CHANNEL_ROWS) + 1,
                                 (i / NUM_CHANNEL_ROWS) * 4 + 2);
            gridChannels.setCell(colMax[i], (i % NUM_CHANNEL_ROWS) + 1,
                                 (i / NUM_CHANNEL_ROWS) * 4 + 3);
        }

        btnMark.setSelectable(true);
        btnMark.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnMark.setBackgroundColor(mxgui::darkGrey);

        btnStop.setSelectable(true);
        btnStop.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnStop.setBackgroundColor(mxgui::darkGrey);

        btnReset.setSelectable(true);
        btnReset.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btnReset.setBackgroundColor(mxgui::darkGrey);

        gridButtons.setCell(&btnMark, 0);
        gridButtons.setCell(&btnReset, 1);
        gridButtons.setCell(&btnStop, 2);
        gridButtons.setDrawBorder(true);

        root.addView(&gridTitle, 0.8);
        root.addView(&gridChannels, 10);
        root.addView(&gridButtons, 1);
    }

    ~EnergyScanScreen()
    {
        for (unsigned int i = 0; i < NUM_COLS; i++)
        {
            delete colTitles[i];
        }

        for (unsigned int i = 0; i < NUM_CHANNELS; ++i)
        {
            delete colNames[i];
            delete colCurrent[i];
            delete colMin[i];
            delete colMax[i];
        }
    }

    void updateScan(array<int, NUM_CHANNELS> scan)
    {
        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            chStats[i].add(scan[i]);

            StatsResult r = chStats[i].getStats();

            absMax = std::max(r.maxValue, absMax);
            absMin = std::min(r.minValue, absMin);
        }

        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            StatsResult r = chStats[i].getStats();

            colMin[i]->setText(to_string((int)r.minValue));
            setColor(colMin[i], r.minValue);

            colCurrent[i]->setText(to_string(scan[i]));
            setColor(colCurrent[i], scan[i]);

            colMax[i]->setText(to_string((int)r.maxValue));
            setColor(colMax[i], r.maxValue);
        }
    }

    void resetStats()
    {
        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            chStats[i].reset();
        }

        absMax = std::numeric_limits<float>::lowest();
        absMin = std::numeric_limits<float>::max();
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

    VerticalLayout root{5};

    TextView btnMark{"Mark Log (1)"};
    TextView btnStop{"Stop"};
    TextView btnReset{"Reset"};

private:
    void setColor(TextView* tv, float val)
    {
        float delta = absMax - absMin;
        if (val >= absMax - delta * COLOR_PERCENTILE)
        {
            tv->setTextColor(mxgui::green);
        }
        else if (val <= absMin + delta * COLOR_PERCENTILE)
        {
            tv->setTextColor(0xFDC0);  // Light orange
        }
        else
        {
            tv->setTextColor(mxgui::white);
        }
    }
    float absMax = std::numeric_limits<float>::lowest();
    float absMin = std::numeric_limits<float>::max();

    GridLayout gridChannels{NUM_CHANNEL_ROWS + 1, NUM_COLS};
    GridLayout gridTitle{1, 2};
    GridLayout gridButtons{1, 3};

    TextView tvLogStatus{"SD ERR"};
    TextView tvTitle{"Energy Scan"};

    array<std::string, NUM_COLS / 2> titles = {"CH", "Curr", "Min", "Max"};
    array<TextView*, NUM_COLS> colTitles;

    array<TextView*, NUM_CHANNELS> colNames;
    array<TextView*, NUM_CHANNELS> colCurrent;
    array<TextView*, NUM_CHANNELS> colMin;
    array<TextView*, NUM_CHANNELS> colMax;

    array<Stats, NUM_CHANNELS> chStats;
};

}  // namespace Boardcore
