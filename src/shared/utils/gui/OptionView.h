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

#include <map>
#include <string>

#include "GridLayout.h"
#include "TextView.h"

/**
 * @brief View used to display an option list, so the user can select one by
 * clicking on it.
 */
class OptionView : public View
{
public:
    using OnOptionChosenListener = std::function<void(unsigned int id)>;

    OptionView(std::string name, std::map<uint8_t, std::string> options,
               uint8_t default_option = 0, uint8_t num_cols = 4)
        : tv_title(new TextView(name))
    {
        uint8_t num_rows = options.size() / num_cols;
        if (options.size() % num_cols != 0)
        {
            ++num_rows;
        }

        grid_options = new GridLayout(num_rows, num_cols);

        unsigned int grid_pos = 0;
        unsigned int i        = 0;
        for (auto it = options.begin(); it != options.end(); it++)
        {
            using namespace std::placeholders;

            TextView* tv_opt = new TextView(it->second);
            tv_opt->setSelectable(true);
            tv_opt->addOnInteractionListener(
                std::bind(&OptionView::onOptionClick, this, _1, _2));
            tv_opt->setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

            map_options[tv_opt] = it->first;

            grid_options->setCell(tv_opt, grid_pos++);

            if (i == default_option)
            {
                selectOption(tv_opt);
            }

            ++i;
        }
    }

    virtual ~OptionView()
    {
        delete grid_options;
        delete tv_title;
        for (auto it = map_options.begin(); it != map_options.end(); it++)
        {
            delete it->first;
        }
    }

    void setBounds(Bounds bounds) override
    {
        View::setBounds(bounds);

        Bounds title_bounds      = getBounds();
        title_bounds.size.height = tv_title->getFont().getHeight();
        tv_title->setBounds(title_bounds);

        Bounds grid_bounds = getBounds();
        grid_bounds.pos.y  = getBounds().pos.y + title_bounds.size.height + 1;
        grid_bounds.size.height =
            getBounds().size.height - title_bounds.size.height;

        grid_options->setBounds(grid_bounds);
    }

    /**
     * @brief Adds a callback to be called each time an option is selected by
     * the user
     */
    void addOnOptionChosenListener(OnOptionChosenListener listener)
    {
        opt_listeners.push_back(listener);
    }

    std::vector<View*> getChilds() override
    {
        std::vector<View*> out;
        out.push_back(tv_title);
        out.push_back(grid_options);

        return out;
    }

    void draw(mxgui::DrawingContext& dc) override
    {
        View::draw(dc);
        tv_title->draw(dc);
        grid_options->draw(dc);
    }

private:
    void selectOption(TextView* opt)
    {
        if (selected_option != nullptr)
        {
            selected_option->setBackgroundColor(col_bg_normal);
        }

        selected_option = opt;

        selected_option->setBackgroundColor(col_bg_highlight);
    }

    void onOptionClick(View* option, Interaction action)
    {
        if (action == Interaction::CLICK)
        {
            TextView* textview = static_cast<TextView*>(option);

            selectOption(textview);

            for (auto l : opt_listeners)
            {
                if (l != nullptr)
                {
                    l(map_options[textview]);
                }
            }
        }
    }

    TextView* tv_title;
    mxgui::Color col_bg_normal    = mxgui::black;
    mxgui::Color col_bg_highlight = mxgui::blue;

    GridLayout* grid_options;
    std::map<TextView*, uint8_t> map_options;

    std::vector<OnOptionChosenListener> opt_listeners;

    TextView* selected_option = nullptr;
};