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
 * @brief Displays childs in a numRows*numCols grid
 */
class GridLayout : public View
{
public:
    using GridPosition = std::pair<uint8_t, uint8_t>;

    /**
     * @brief Creates a new GridLayout
     *
     * @param numRows Number of rows
     * @param numCols Number of columns
     * @param spacing Distance in pixels between each cell
     */
    GridLayout(uint8_t numRows, uint8_t numCols, short int spacing = 0)
        : View(), numRows(numRows), numCols(numCols), spacing(spacing)
    {
    }

    virtual ~GridLayout() {}

    void setCell(View* child, unsigned int position)
    {
        uint8_t col = position % numCols;
        uint8_t row = (position - col) / numCols;

        setCell(child, row, col);
    }

    void setCell(View* child, uint8_t row, uint8_t col)
    {
        if (row >= numRows || col >= numCols)
        {
            return;
        }

        mapChilds[GridPosition(row, col)] = child;

        updateChildBounds();
        invalidate();
    }

    View* getCell(uint8_t row, uint8_t col)
    {
        GridPosition pos(row, col);

        if (mapChilds.count(pos) > 0)
            return mapChilds[pos];
        else
            return nullptr;
    }

    View* getCell(unsigned int position)
    {
        uint8_t col = position % numCols;
        uint8_t row = position - col / numRows;

        return getCell(row, col);
    }

    void clearCell(View* child)
    {
        for (auto it = mapChilds.begin(); it != mapChilds.end(); it++)
        {
            if (it->second == child)
            {
                mapChilds.erase(it);
                break;
            }
        }

        invalidate();
    }

    /**
     * @brief Wether to draw the borders of each cell or not
     *
     * @param drawBorder True: draw the border
     * @param color Border color
     */
    void setDrawBorder(bool drawBorder, mxgui::Color color = mxgui::white)
    {
        this->drawBorder = drawBorder;
        borderColor      = color;

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

        for (uint8_t row = 0; row < numRows; ++row)
        {
            for (uint8_t col = 0; col < numCols; ++col)
            {
                GridPosition pos(row, col);

                bool childSelected = false;
                if (mapChilds.count(pos) > 0)
                {
                    mapChilds[pos]->draw(context);
                    childSelected = mapChilds[pos]->isSelected();
                }

                if (drawBorder && !childSelected)
                {
                    context.drawRectangle(mapChildBounds[pos].topLeft(),
                                          mapChildBounds[pos].bottomRight(),
                                          borderColor);
                }
            }
        }
    }

    uint8_t getRows() { return numRows; }

    uint8_t getCols() { return numCols; }

    std::vector<View*> getChilds() override
    {
        std::vector<View*> out;
        for (auto it = mapChilds.begin(); it != mapChilds.end(); it++)
        {
            out.push_back(it->second);
        }
        return out;
    }

private:
    void updateChildBounds()
    {
        Bounds bounds = getBounds();
        Bounds childBounds;

        childBounds.size.width = std::max(
            0, (bounds.size.width - (numCols + 1) * spacing) / numCols);
        childBounds.size.height = std::max(
            0, (bounds.size.height - (numRows + 1) * spacing) / numRows);

        for (uint8_t row = 0; row < numRows; ++row)
        {
            for (uint8_t col = 0; col < numCols; ++col)
            {
                GridPosition pos(row, col);

                childBounds.pos.x = bounds.pos.x + (col + 1) * spacing +
                                    col * childBounds.size.width;
                childBounds.pos.y = bounds.pos.y + (row + 1) * spacing +
                                    row * childBounds.size.height;

                mapChildBounds[pos] = childBounds;
                if (mapChilds.count(pos) > 0)
                {
                    mapChilds[pos]->setBounds(childBounds);
                }
            }
        }
    }

    uint8_t numRows;
    uint8_t numCols;
    short int spacing;

    bool drawBorder          = false;
    mxgui::Color borderColor = mxgui::white;

    std::map<GridPosition, View*> mapChilds;
    std::map<GridPosition, Bounds> mapChildBounds;
};

}  // namespace Boardcore
