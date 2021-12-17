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

#include <mxgui/misc_inst.h>

#include <map>

#include "View.h"

namespace Boardcore
{

/**
 * @brief Displays childs in a num_rows*num_cols grid
 */
class GridLayout : public View
{
public:
    using GridPosition = std::pair<uint8_t, uint8_t>;

    /**
     * @brief Creates a new GridLayout
     *
     * @param num_rows Number of rows
     * @param num_cols Number of columns
     * @param spacing Distance in pixels between each cell
     */
    GridLayout(uint8_t num_rows, uint8_t num_cols, short int spacing = 0)
        : View(), num_rows(num_rows), num_cols(num_cols), spacing(spacing)
    {
    }

    virtual ~GridLayout() {}

    void setCell(View* child, unsigned int position)
    {
        uint8_t col = position % num_cols;
        uint8_t row = (position - col) / num_cols;

        setCell(child, row, col);
    }

    void setCell(View* child, uint8_t row, uint8_t col)
    {
        if (row >= num_rows || col >= num_cols)
        {
            return;
        }

        map_childs[GridPosition(row, col)] = child;

        updateChildBounds();
        invalidate();
    }

    View* getCell(uint8_t row, uint8_t col)
    {
        GridPosition pos(row, col);

        if (map_childs.count(pos) > 0)
            return map_childs[pos];
        else
            return nullptr;
    }

    View* getCell(unsigned int position)
    {
        uint8_t col = position % num_cols;
        uint8_t row = position - col / num_rows;

        return getCell(row, col);
    }

    void clearCell(View* child)
    {
        for (auto it = map_childs.begin(); it != map_childs.end(); it++)
        {
            if (it->second == child)
            {
                map_childs.erase(it);
                break;
            }
        }

        invalidate();
    }

    /**
     * @brief Wether to draw the borders of each cell or not
     *
     * @param draw_border True: draw the border
     * @param color Border color
     */
    void setDrawBorder(bool draw_border, mxgui::Color color = mxgui::white)
    {
        this->draw_border = draw_border;
        border_color      = color;

        invalidate();
    }

    void setBounds(Bounds bounds) override
    {
        View::setBounds(bounds);

        updateChildBounds();
    }

    virtual void draw(mxgui::DrawingContext& context) override
    {
        View::draw(context);

        for (uint8_t row = 0; row < num_rows; ++row)
        {
            for (uint8_t col = 0; col < num_cols; ++col)
            {
                GridPosition pos(row, col);

                bool child_selected = false;
                if (map_childs.count(pos) > 0)
                {
                    map_childs[pos]->draw(context);
                    child_selected = map_childs[pos]->isSelected();
                }

                if (draw_border && !child_selected)
                {
                    context.drawRectangle(map_child_bounds[pos].topLeft(),
                                          map_child_bounds[pos].bottomRight(),
                                          border_color);
                }
            }
        }
    }

    uint8_t getRows() { return num_rows; }

    uint8_t getCols() { return num_cols; }

    std::vector<View*> getChilds() override
    {
        std::vector<View*> out;
        for (auto it = map_childs.begin(); it != map_childs.end(); it++)
        {
            out.push_back(it->second);
        }
        return out;
    }

private:
    void updateChildBounds()
    {
        Bounds bounds = getBounds();
        Bounds child_bounds;

        child_bounds.size.width = std::max(
            0, (bounds.size.width - (num_cols + 1) * spacing) / num_cols);
        child_bounds.size.height = std::max(
            0, (bounds.size.height - (num_rows + 1) * spacing) / num_rows);

        for (uint8_t row = 0; row < num_rows; ++row)
        {
            for (uint8_t col = 0; col < num_cols; ++col)
            {
                GridPosition pos(row, col);

                child_bounds.pos.x = bounds.pos.x + (col + 1) * spacing +
                                     col * child_bounds.size.width;
                child_bounds.pos.y = bounds.pos.y + (row + 1) * spacing +
                                     row * child_bounds.size.height;

                map_child_bounds[pos] = child_bounds;
                if (map_childs.count(pos) > 0)
                {
                    map_childs[pos]->setBounds(child_bounds);
                }
            }
        }
    }

    uint8_t num_rows;
    uint8_t num_cols;
    short int spacing;

    bool draw_border          = false;
    mxgui::Color border_color = mxgui::white;

    std::map<GridPosition, View*> map_childs;
    std::map<GridPosition, Bounds> map_child_bounds;
};

}  // namespace Boardcore
