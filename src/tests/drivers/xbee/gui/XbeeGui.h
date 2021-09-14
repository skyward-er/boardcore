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

#include <cstdint>
#include <functional>

#include "ConfigScreen.h"
#include "StatusScreen.h"
#include "EnergyScanScreen.h"
#include "EndScreen.h"
#include "RespectScreen.h"
#include "utils/gui/ScreenManager.h"

class XbeeGUI
{
public:
    enum Screens : uint8_t
    {
        SCREEN_CONFIG,
        SCREEN_STATUS,
        SCREEN_ENERGYSCAN,
        SCREEN_END,
        SCREEN_RESPECT
    };

    XbeeGUI()
        : screen_manager(mxgui::DisplayManager::instance(), 4)
    {
        screen_manager.addScreen(SCREEN_CONFIG, &screen_config.root);
        screen_manager.addScreen(SCREEN_STATUS, &screen_status.root);
        screen_manager.addScreen(SCREEN_ENERGYSCAN, &screen_energy.root);
        screen_manager.addScreen(SCREEN_END, &screen_end.root);
        screen_manager.addScreen(SCREEN_RESPECT, &screen_respect.root);

        screen_manager.start();
    }

    ~XbeeGUI()
    {
        screen_manager.stop();
    }

    ScreenManager screen_manager;

    ConfigScreen screen_config{};
    StatusScreen screen_status{};
    EnergyScanScreen screen_energy{};
    EndScreen screen_end{};
    RespectScreen screen_respect{};
};