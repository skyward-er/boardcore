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

#include <mxgui/display.h>
#include <mxgui/misc_inst.h>
#include <utils/Debug.h>

#include <functional>

#include "Misc.h"

namespace Boardcore
{

struct Size
{
    short int width;
    short int height;
};

struct Position
{
    short int x;
    short int y;

    operator mxgui::Point() { return mxgui::Point(x, y); }

    Position operator+(const Position& other)
    {
        return {short(x + other.x), short(y + other.y)};
    }

    Position operator-(const Position& other)
    {
        return {short(x - other.x), short(y - other.y)};
    }

    void operator+=(const Position& other)
    {
        x += other.x;
        y += other.y;
    }

    void operator-=(const Position& other)
    {
        x -= other.x;
        y -= other.y;
    }

    void print() { TRACE("pos: %d %d\n", x, y); }
};

struct Bounds
{
    Position pos;
    Size size;

    Position topLeft() { return {pos.x, pos.y}; }
    Position topRight() { return {short(pos.x + size.width - 1), pos.y}; }

    Position bottomRight()
    {
        return {short(pos.x + size.width - 1), short(pos.y + size.height - 1)};
    }

    Position bottomLeft()
    {
        return {pos.x, short(pos.y + short(size.height - 1))};
    }

    void print()
    {
        TRACE("Bounds: p:{%d %d} s:{%d %d}\n", pos.x, pos.y, size.width,
              size.height);
    }
};

enum class VertAlignment
{
    TOP,
    CENTER,
    BOTTOM
};

enum class HorizAlignment
{
    LEFT,
    CENTER,
    RIGHT
};

enum class Interaction
{
    BTN_DOWN,
    BTN_UP,
    CLICK,
    LONG_CLICK
};

/**
 * @brief Base class for anything that can be drawn on the screen and interacted
 * with
 */
class View
{
public:
    using OnInteractionListener = std::function<void(View*, Interaction)>;

    View() {}
    virtual ~View() {}

    /**
     * @brief Sets the bounds in which the view will be drawn
     */
    virtual void setBounds(Bounds bounds)
    {
        this->bounds = bounds;
        invalidate();
    }

    Bounds getBounds() { return bounds; }

    /**
     * @brief Signal that what has been previously drawn is now invalid and has
     * to be redrawn.
     */
    void invalidate()
    {
        if (!invalidated)
        {
            invalidated = true;
        }
    }

    /**
     * @brief Invalidate the view tree which has this view as a root
     */
    void invalidateTree()
    {
        invalidate();

        std::vector<View*> childs = getChilds();
        for (auto child : childs)
        {
            child->invalidateTree();
        }
    }

    /**
     * @brief Draw the view in its bounds
     *
     * @param dc Reference to a drawingcontext
     */
    virtual void draw(mxgui::DrawingContext& dc)
    {
        if (invalidated)
        {
            invalidated = false;

            dc.clear(bounds.topLeft(), bounds.bottomRight(),
                     getBackgroundColor());

            dc.drawRectangle(bounds.topLeft(), bounds.bottomRight(),
                             selected ? colSelected : getBackgroundColor());
        }
    }

    /**
     * @brief Returns the list of childs
     *
     * @return std::vector<View*>
     */
    virtual std::vector<View*> getChilds()
    {
        // Default behavious is no childs
        return {};
    }

    virtual void setBackgroundColor(mxgui::Color color)
    {
        if (color != colBg)
        {
            colBg = color;
            invalidate();
        }
    }

    virtual mxgui::Color getBackgroundColor() { return colBg; }

    void setSelectedColor(mxgui::Color color)
    {
        colSelected = color;
        invalidate();
    }

    void setSelectable(bool selectable) { this->selectable = selectable; }

    bool isSelectable() { return selectable; }

    void setSelected(bool selected)
    {
        this->selected = selected;
        invalidate();
    }

    bool isSelected() { return selected && selectable; }

    void addOnInteractionListener(OnInteractionListener listener)
    {
        listeners.push_back(listener);
    }

    virtual void performInteraction(Interaction action)
    {
        for (auto lst : listeners)
        {
            if (lst)
            {
                lst(this, action);
            }
        }
    }

protected:
    bool isInvalidated() { return invalidated; }

private:
    mxgui::Color colSelected = mxgui::red;
    mxgui::Color colBg       = mxgui::black;

    std::vector<OnInteractionListener> listeners;

    bool selected    = false;
    bool selectable  = false;
    bool invalidated = true;

    Bounds bounds{{0, 0}, {0, 0}};

    // Drawing big solid chunks (such as backgrounds) takes a lot of time: do
    // not redraw if not necessary
    bool wasSelected         = false;
    mxgui::Color lastBgColor = colBg;
};

}  // namespace Boardcore
