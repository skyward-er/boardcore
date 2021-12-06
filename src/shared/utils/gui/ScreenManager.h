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

#include <miosix.h>

#include <deque>
#include <map>

#include "NavController.h"
#include "View.h"

namespace Boardcore
{

/**
 * @brief UI Thread: Manages multiple view trees ("Screen") and draws the active
 * one at the provided refresh rate
 */
class ScreenManager : public ActiveObject
{
public:
    ScreenManager(mxgui::DisplayManager& display, unsigned int refresh_rate)
        : dc(display.getDisplay()), refresh_interval(1000 / refresh_rate)
    {
    }

    void showScreen(uint8_t id)
    {
        active_screen = id;
        controller.updateViewTree(screens[id]);
    }

    uint8_t getScreen() { return active_screen; }

    void addScreen(uint8_t id, View* root)
    {
        screens[id] = root;

        root->setBounds({{0, 0}, {dc.getWidth(), dc.getHeight()}});

        if (screens.size() == 1)
        {
            showScreen(id);
        }
    }

    void onButtonPress(ButtonPress press) { controller.onButtonPress(press); }

    mxgui::DrawingContext& getDrawingContext() { return dc; }

protected:
    void run() override
    {
        uint8_t last_screen = 0;
        while (!shouldStop())
        {
            if (active_screen != last_screen)
            {
                last_screen = active_screen;
                dc.clear(mxgui::black);
                screens[active_screen]->invalidateTree();
            }

            long long start = miosix::getTick();

            drawViewTree(screens[active_screen], dc);

            Thread::sleepUntil(start + refresh_interval);
        }
    }

private:
    /**
     * @brief Draws the provided view tree on screen
     *
     * @param root Root of the view tree
     * @param dc Drawing context
     */
    void drawViewTree(View* root, mxgui::DrawingContext& dc)
    {
        // Avoid recursion
        std::deque<View*> views_dc;
        views_dc.push_back(root);

        while (views_dc.size() != 0)
        {
            View* view = views_dc.front();
            views_dc.pop_front();

            view->draw(dc);

            for (View* c : view->getChilds())
            {
                views_dc.push_back(c);
            }
        }
    }

    mxgui::DrawingContext dc;

    unsigned int refresh_interval;

    NavController controller;
    std::map<uint8_t, View*> screens;

    uint8_t active_screen = 0;
};

}  // namespace Boardcore
