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
        tv_title.setFont(mxgui::miscFixedBold);
        tv_title.setTextColor(mxgui::black);
        tv_title.setBackgroundColor(mxgui::green);
        tv_title.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tv_log_status.setFont(mxgui::miscFixedBold);
        tv_log_status.setTextColor(mxgui::white);
        tv_log_status.setBackgroundColor(mxgui::red);
        tv_log_status.setAlignment(HorizAlignment::CENTER,
                                   VertAlignment::CENTER);

        grid_title.setCell(&tv_title, 0, 0);
        grid_title.setCell(&tv_log_status, 0, 1);

        for (unsigned int i = 0; i < NUM_COLS; i++)
        {
            col_titles[i] = new TextView(titles[i % (NUM_COLS / 2)]);
            col_titles[i]->setTextColor(mxgui::blue);

            grid_channels.setCell(col_titles[i], 0, i);
        }

        mxgui::Color pink(0xFC59);
        for (unsigned int i = 0; i < NUM_CHANNELS; ++i)
        {
            char buf[3];
            snprintf(buf, 3, "%02u", i);

            col_names[i] = new TextView(buf);
            col_names[i]->setTextColor(pink);

            col_current[i] = new TextView("-40");
            col_min[i]     = new TextView("-40");
            col_max[i]     = new TextView("-40");

            grid_channels.setCell(col_names[i], (i % NUM_CHANNEL_ROWS) + 1,
                                  (i / NUM_CHANNEL_ROWS) * 4 + 0);
            grid_channels.setCell(col_current[i], (i % NUM_CHANNEL_ROWS) + 1,
                                  (i / NUM_CHANNEL_ROWS) * 4 + 1);
            grid_channels.setCell(col_min[i], (i % NUM_CHANNEL_ROWS) + 1,
                                  (i / NUM_CHANNEL_ROWS) * 4 + 2);
            grid_channels.setCell(col_max[i], (i % NUM_CHANNEL_ROWS) + 1,
                                  (i / NUM_CHANNEL_ROWS) * 4 + 3);
        }

        btn_mark.setSelectable(true);
        btn_mark.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_mark.setBackgroundColor(mxgui::darkGrey);

        btn_stop.setSelectable(true);
        btn_stop.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_stop.setBackgroundColor(mxgui::darkGrey);

        btn_reset.setSelectable(true);
        btn_reset.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        btn_reset.setBackgroundColor(mxgui::darkGrey);

        grid_buttons.setCell(&btn_mark, 0);
        grid_buttons.setCell(&btn_reset, 1);
        grid_buttons.setCell(&btn_stop, 2);
        grid_buttons.setDrawBorder(true);

        root.addView(&grid_title, 0.8);
        root.addView(&grid_channels, 10);
        root.addView(&grid_buttons, 1);
    }

    ~EnergyScanScreen()
    {
        for (unsigned int i = 0; i < NUM_COLS; i++)
        {
            delete col_titles[i];
        }

        for (unsigned int i = 0; i < NUM_CHANNELS; ++i)
        {
            delete col_names[i];
            delete col_current[i];
            delete col_min[i];
            delete col_max[i];
        }
    }

    void updateScan(array<int, NUM_CHANNELS> scan)
    {
        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            ch_stats[i].add(scan[i]);

            StatsResult r = ch_stats[i].getStats();

            abs_max = std::max(r.maxValue, abs_max);
            abs_min = std::min(r.minValue, abs_min);
        }

        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            StatsResult r = ch_stats[i].getStats();

            col_min[i]->setText(to_string((int)r.minValue));
            setColor(col_min[i], r.minValue);

            col_current[i]->setText(to_string(scan[i]));
            setColor(col_current[i], scan[i]);

            col_max[i]->setText(to_string((int)r.maxValue));
            setColor(col_max[i], r.maxValue);
        }
    }

    void resetStats()
    {
        for (unsigned int i = 0; i < NUM_CHANNELS; i++)
        {
            ch_stats[i].reset();
        }

        abs_max = std::numeric_limits<float>::lowest();
        abs_min = std::numeric_limits<float>::max();
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

    VerticalLayout root{5};

    TextView btn_mark{"Mark Log (1)"};
    TextView btn_stop{"Stop"};
    TextView btn_reset{"Reset"};

private:
    void setColor(TextView* tv, float val)
    {
        float delta = abs_max - abs_min;
        if (val >= abs_max - delta * COLOR_PERCENTILE)
        {
            tv->setTextColor(mxgui::green);
        }
        else if (val <= abs_min + delta * COLOR_PERCENTILE)
        {
            tv->setTextColor(0xFDC0);  // Light orange
        }
        else
        {
            tv->setTextColor(mxgui::white);
        }
    }
    float abs_max = std::numeric_limits<float>::lowest();
    float abs_min = std::numeric_limits<float>::max();

    GridLayout grid_channels{NUM_CHANNEL_ROWS + 1, NUM_COLS};
    GridLayout grid_title{1, 2};
    GridLayout grid_buttons{1, 3};

    TextView tv_log_status{"SD ERR"};
    TextView tv_title{"Energy Scan"};

    array<std::string, NUM_COLS / 2> titles = {"CH", "Curr", "Min", "Max"};
    array<TextView*, NUM_COLS> col_titles;

    array<TextView*, NUM_CHANNELS> col_names;
    array<TextView*, NUM_CHANNELS> col_current;
    array<TextView*, NUM_CHANNELS> col_min;
    array<TextView*, NUM_CHANNELS> col_max;

    array<Stats, NUM_CHANNELS> ch_stats;
};

}  // namespace Boardcore
