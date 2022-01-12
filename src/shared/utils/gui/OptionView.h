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

namespace Boardcore
{

/**
 * @brief View used to display an option list, so the user can select one by
 * clicking on it.
 */
class OptionView : public View
{
public:
    using OnOptionChosenListener = std::function<void(unsigned int id)>;

    OptionView(std::string name, std::map<uint8_t, std::string> options,
               uint8_t defaultOption = 0, uint8_t numCols = 4)
        : tvTitle(new TextView(name))
    {
        uint8_t numRows = options.size() / numCols;
        if (options.size() % numCols != 0)
        {
            ++numRows;
        }

        gridOptions = new GridLayout(numRows, numCols);

        unsigned int gridPos = 0;
        unsigned int i       = 0;
        for (auto it = options.begin(); it != options.end(); it++)
        {
            using namespace std::placeholders;

            TextView* tvOpt = new TextView(it->second);
            tvOpt->setSelectable(true);
            tvOpt->addOnInteractionListener(
                std::bind(&OptionView::onOptionClick, this, _1, _2));
            tvOpt->setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

            mapOptions[tvOpt] = it->first;

            gridOptions->setCell(tvOpt, gridPos++);

            if (i == defaultOption)
            {
                selectOption(tvOpt);
            }

            ++i;
        }
    }

    virtual ~OptionView()
    {
        delete gridOptions;
        delete tvTitle;
        for (auto it = mapOptions.begin(); it != mapOptions.end(); it++)
        {
            delete it->first;
        }
    }

    void setBounds(Bounds bounds) override
    {
        View::setBounds(bounds);

        Bounds titleBounds      = getBounds();
        titleBounds.size.height = tvTitle->getFont().getHeight();
        tvTitle->setBounds(titleBounds);

        Bounds gridBounds = getBounds();
        gridBounds.pos.y  = getBounds().pos.y + titleBounds.size.height + 1;
        gridBounds.size.height =
            getBounds().size.height - titleBounds.size.height;

        gridOptions->setBounds(gridBounds);
    }

    /**
     * @brief Adds a callback to be called each time an option is selected by
     * the user
     */
    void addOnOptionChosenListener(OnOptionChosenListener listener)
    {
        optListeners.push_back(listener);
    }

    std::vector<View*> getChilds() override
    {
        std::vector<View*> out;
        out.push_back(tvTitle);
        out.push_back(gridOptions);

        return out;
    }

    void draw(mxgui::DrawingContext& dc) override
    {
        View::draw(dc);
        tvTitle->draw(dc);
        gridOptions->draw(dc);
    }

private:
    void selectOption(TextView* opt)
    {
        if (selectedOption != nullptr)
        {
            selectedOption->setBackgroundColor(colBgNormal);
        }

        selectedOption = opt;

        selectedOption->setBackgroundColor(colBgHighlight);
    }

    void onOptionClick(View* option, Interaction action)
    {
        if (action == Interaction::CLICK)
        {
            TextView* textview = static_cast<TextView*>(option);

            selectOption(textview);

            for (auto l : optListeners)
            {
                if (l != nullptr)
                {
                    l(mapOptions[textview]);
                }
            }
        }
    }

    TextView* tvTitle;
    mxgui::Color colBgNormal    = mxgui::black;
    mxgui::Color colBgHighlight = mxgui::blue;

    GridLayout* gridOptions;
    std::map<TextView*, uint8_t> mapOptions;

    std::vector<OnOptionChosenListener> optListeners;

    TextView* selectedOption = nullptr;
};

}  // namespace Boardcore
