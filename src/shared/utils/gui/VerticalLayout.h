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

#include "View.h"

namespace Boardcore
{

/**
 * @brief Positions the childs in a vertical grid. The height of each child is
 * dictated by its weight parameter
 */
class VerticalLayout : public View
{
public:
    /**
     * @brief Creates a new VerticalLayout
     *
     * @param spacing Vertical space between each child in pixels
     */
    VerticalLayout(short int spacing = 0) : View(), spacing(spacing) {}

    /**
     * @brief Adds a view to the layout.
     *
     * @param view Pointer to the view to be added
     * @param weight Relative weight of the view to determine its display height
     */
    void addView(View* view, float weight)
    {
        if (weight < 0)
            weight = 0;
        childs.push_back({weight, view});
        weigth_sum += weight;

        // Recalculated child bounds
        updateChildBounds();

        // Redraw
        invalidate();
    }

    void setBounds(Bounds bounds) override
    {
        View::setBounds(bounds);

        updateChildBounds();
    }

    void draw(mxgui::DrawingContext& dc) override
    {
        View::draw(dc);

        for (auto view : childs)
        {
            view.drawable->draw(dc);
        }
    }

    virtual std::vector<View*> getChilds() override
    {
        std::vector<View*> out;
        out.reserve(childs.size());
        for (auto v : childs)
        {
            out.push_back(v.drawable);
        }
        return out;
    }

private:
    void updateChildBounds()
    {
        if (weigth_sum > 0)
        {
            short int aval_height =
                getBounds().size.height - (childs.size() - 1) * spacing;

            short int y = 0;

            for (auto view : childs)
            {
                Bounds view_bounds{{0, y}, {getBounds().size.width, 0}};

                view_bounds.size.height =
                    aval_height / weigth_sum * view.vert_weight;
                view_bounds.pos += getBounds().pos;

                view.drawable->setBounds(view_bounds);

                y += view_bounds.size.height + spacing;
            }
        }
    }

    struct Child
    {
        float vert_weight;
        View* drawable;
    };

    float weigth_sum = 0;
    short int spacing;
    std::vector<Child> childs;
};

}  // namespace Boardcore
