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

#include <vector>

#include "Debug.h"
#include "TextView.h"
#include "View.h"
#include "utils/ButtonHandler.h"

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
        if (selected_index < vec_selectable.size())
        {
            vec_selectable.at(selected_index)->setSelected(false);
        }

        vec_selectable.clear();
        selected_index = 0;

        updateSelectableViews(root);

        if (vec_selectable.size() > 0)
        {
            vec_selectable.at(0)->setSelected(true);
        }
    }

    void onButtonPress(ButtonPress press)
    {
        switch (press)
        {
            case ButtonPress::SHORT:
                if (vec_selectable.size() > 0)
                {
                    selectNext();
                }
                break;
            case ButtonPress::DOWN:
                if (selected_index < vec_selectable.size())
                {
                    vec_selectable.at(selected_index)
                        ->performInteraction(Interaction::BTN_DOWN);
                }
                break;
            case ButtonPress::UP:
                if (selected_index < vec_selectable.size())
                {
                    vec_selectable.at(selected_index)
                        ->performInteraction(Interaction::BTN_UP);
                }
                break;
            case ButtonPress::LONG:
                if (selected_index < vec_selectable.size())
                {
                    vec_selectable.at(selected_index)
                        ->performInteraction(Interaction::CLICK);
                }
                break;
            case ButtonPress::VERY_LONG:
                if (selected_index < vec_selectable.size())
                {
                    vec_selectable.at(selected_index)
                        ->performInteraction(Interaction::LONG_CLICK);
                }
                break;
            default:
                break;
        }
    }

private:
    void selectNext()
    {
        // Deselect old drawble
        if (selected_index < vec_selectable.size())
        {
            vec_selectable.at(selected_index)->setSelected(false);
        }

        if (vec_selectable.size() > 0)
        {
            selected_index = (selected_index + 1) % vec_selectable.size();

            vec_selectable.at(selected_index)->setSelected(true);

            TextView* text =
                dynamic_cast<TextView*>(vec_selectable.at(selected_index));

            if (text)
            {
                TRACE("[NavController] %s\n", text->getText().c_str());
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
                vec_selectable.push_back(child);
            }
            updateSelectableViews(child);
        }
    }

    unsigned int selected_index = 0;
    std::vector<View*> vec_selectable;
};