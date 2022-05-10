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

#include <diagnostic/PrintLogger.h>
#include <utils/ButtonHandler/ButtonHandler.h>
#include <utils/Debug.h>

#include <vector>

#include "TextView.h"
#include "View.h"

namespace Boardcore
{

/**
 * @brief UI navigation controller: listens for button clicks and dispatches the
 * interactions to the view tree.
 */
class NavController
{
public:
    NavController() {}

    /**
     * @brief
     *
     * @param root
     */
    void updateViewTree(View* root)
    {
        if (selectedIndex < vecSelectable.size())
        {
            vecSelectable.at(selectedIndex)->setSelected(false);
        }

        vecSelectable.clear();
        selectedIndex = 0;

        updateSelectableViews(root);

        if (vecSelectable.size() > 0)
        {
            vecSelectable.at(0)->setSelected(true);
        }
    }

    void onButtonEvent(ButtonEvent press)
    {
        switch (press)
        {
            case ButtonEvent::PRESSED:
                if (selectedIndex < vecSelectable.size())
                    vecSelectable.at(selectedIndex)
                        ->performInteraction(Interaction::BTN_DOWN);
                if (vecSelectable.size() > 0)
                    selectNext();
                break;
            case ButtonEvent::SHORT_PRESS:
                if (selectedIndex < vecSelectable.size())
                    vecSelectable.at(selectedIndex)
                        ->performInteraction(Interaction::BTN_UP);
                if (vecSelectable.size() > 0)
                    selectNext();
                break;
            case ButtonEvent::LONG_PRESS:
                if (selectedIndex < vecSelectable.size())
                    vecSelectable.at(selectedIndex)
                        ->performInteraction(Interaction::CLICK);
                if (vecSelectable.size() > 0)
                    selectNext();
                break;
            case ButtonEvent::VERY_LONG_PRESS:
                if (selectedIndex < vecSelectable.size())
                    vecSelectable.at(selectedIndex)
                        ->performInteraction(Interaction::LONG_CLICK);
                if (vecSelectable.size() > 0)
                    selectNext();
                break;
            default:
                break;
        }
    }

private:
    void selectNext()
    {
        // Deselect old drawble
        if (selectedIndex < vecSelectable.size())
        {
            vecSelectable.at(selectedIndex)->setSelected(false);
        }

        if (vecSelectable.size() > 0)
        {
            selectedIndex = (selectedIndex + 1) % vecSelectable.size();

            vecSelectable.at(selectedIndex)->setSelected(true);

            TextView* text =
                dynamic_cast<TextView*>(vecSelectable.at(selectedIndex));

            if (text)
            {
                LOG_DEBUG(logger, "{}", text->getText().c_str());
            }
        }
    }

    void updateSelectableViews(View* root)
    {
        std::vector<View*> childs = root->getChilds();
        for (auto child : childs)
        {
            if (child->isSelectable())
            {
                vecSelectable.push_back(child);
            }
            updateSelectableViews(child);
        }
    }

    unsigned int selectedIndex = 0;
    std::vector<View*> vecSelectable;

    PrintLogger logger = Logging::getLogger("navcontroller");
};

}  // namespace Boardcore
