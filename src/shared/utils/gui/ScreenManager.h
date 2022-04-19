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
    ScreenManager(mxgui::DisplayManager& display, unsigned int refreshRate)
        : dc(display.getDisplay()), refreshInterval(1000 / refreshRate)
    {
    }

    void showScreen(uint8_t id)
    {
        activeScreen = id;
        controller.updateViewTree(screens[id]);
    }

    uint8_t getScreen() { return activeScreen; }

    void addScreen(uint8_t id, View* root)
    {
        screens[id] = root;

        root->setBounds({{0, 0}, {dc.getWidth(), dc.getHeight()}});

        if (screens.size() == 1)
        {
            showScreen(id);
        }
    }

    void onButtonEvent(ButtonEvent press) { controller.onButtonEvent(press); }

    mxgui::DrawingContext& getDrawingContext() { return dc; }

protected:
    void run() override
    {
        uint8_t lastScreen = 0;
        while (!shouldStop())
        {
            if (activeScreen != lastScreen)
            {
                lastScreen = activeScreen;
                dc.clear(mxgui::black);
                screens[activeScreen]->invalidateTree();
            }

            long long start = miosix::getTick();

            drawViewTree(screens[activeScreen], dc);

            miosix::Thread::sleepUntil(start + refreshInterval);
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
        std::deque<View*> viewsDc;
        viewsDc.push_back(root);

        while (viewsDc.size() != 0)
        {
            View* view = viewsDc.front();
            viewsDc.pop_front();

            view->draw(dc);

            for (View* c : view->getChilds())
            {
                viewsDc.push_back(c);
            }
        }
    }

    mxgui::DrawingContext dc;

    unsigned int refreshInterval;

    NavController controller;
    std::map<uint8_t, View*> screens;

    uint8_t activeScreen = 0;
};

}  // namespace Boardcore
