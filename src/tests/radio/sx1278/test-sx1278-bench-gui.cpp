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

#include "gui/GUI.h"

// Include body of the test
#include "test-sx1278-bench.cpp"

using namespace mxgui;

GUI *gui = nullptr;

void initGUI()
{
    // TODO: This should be in bsp
    using GpioUserBtn = Gpio<GPIOA_BASE, 0>;
    GpioUserBtn::mode(Mode::INPUT_PULL_DOWN);

    gui = new GUI();

    ButtonHandler::getInstance().registerButtonCallback(
        GpioUserBtn::getPin(),
        [](auto event) { gui->screen_manager.onButtonEvent(event); });
}

int main()
{
    initBoard();
    initGUI();
    if (!initRadio())
    {
        while (1)
            ;
    }

    // Set display to ready
    gui->stats_screen.updateReady();

    // Initialize backgrounds threads
    spawnThreads();

    while (1)
    {
        StatsScreen::Data data = {
            stats.txBitrate(), stats.rxBitrate(), stats.corrupted_count,
            stats.sent_count,  stats.recv_count,  stats.rssi,
            stats.fei,         stats.snr};

        gui->stats_screen.updateStats(data);
        Thread::sleep(100);
    }
}
