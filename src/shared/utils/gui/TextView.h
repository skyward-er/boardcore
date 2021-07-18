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

#include "View.h"

/**
 * @brief Simple view to display text on screen.
 *
 */
class TextView : public View
{
public:
    /**
     * @brief Creates a new TextView
     *
     * @param text Text
     * @param text_color Text color
     * @param padding Spacing from view bounds in pixels
     */
    TextView(std::string text = "", mxgui::Color text_color = mxgui::white,
             uint8_t padding = 1)
        : View(), text(text), col_text(text_color), padding(padding)
    {
    }

    void setVerticalAlignment(VertAlignment alignment)
    {
        vert_align = alignment;
        invalidate();
    }

    void setHorizontalAlignment(HorizAlignment alignment)
    {
        horiz_align = alignment;
        invalidate();
    }

    void setAlignment(HorizAlignment horiz_align, VertAlignment vert_align)
    {
        setVerticalAlignment(vert_align);
        setHorizontalAlignment(horiz_align);
    }

    void setText(std::string text)
    {
        if (text != this->text)
        {
            invalidate();
            this->text = text;
        }
    }

    std::string getText() { return text; }

    virtual void setTextColor(mxgui::Color color)
    {
        if (color != col_text)
        {
            this->col_text = color;
            invalidate();
        }
    }

    mxgui::Color getTextColor() { return col_text; }

    virtual void draw(mxgui::DrawingContext& dc) override
    {
        if (isInvalidated())
        {
            View::draw(dc);

            Bounds padded_bounds = getBounds();
            padded_bounds.pos += Position{padding, padding};
            padded_bounds.size.height -= padding;
            padded_bounds.size.width -= padding;

            Position top_left = padded_bounds.topLeft();
            switch (vert_align)
            {
                case VertAlignment::BOTTOM:
                    top_left.y =
                        padded_bounds.bottomLeft().y - dc.getFont().getHeight();
                    break;
                case VertAlignment::CENTER:
                    top_left.y =
                        padded_bounds.pos.y +
                        (padded_bounds.size.height - dc.getFont().getHeight()) /
                            2;
                    break;
                case VertAlignment::TOP:
                    break;
            }

            switch (horiz_align)
            {
                case HorizAlignment::RIGHT:
                    top_left.x =
                        padded_bounds.topRight().x - getTextWidth(text);
                    break;
                case HorizAlignment::CENTER:
                    top_left.x =
                        padded_bounds.pos.x +
                        (padded_bounds.size.width - getTextWidth(text)) / 2;
                    break;
                case HorizAlignment::LEFT:
                    break;
            }

            mxgui::Font old_font = dc.getFont();
            dc.setFont(font);

            dc.setTextColor(getTextColor(), getBackgroundColor());
            dc.clippedWrite(top_left, padded_bounds.topLeft(),
                            padded_bounds.bottomRight(), text);

            dc.setFont(old_font);
        }
    }

    void setFont(mxgui::Font font)
    {
        this->font = font;
        invalidate();
    }

    mxgui::Font getFont() { return font; }

protected:
    short int getTextWidth(std::string text)
    {
        short int w = 0;
        for (char c : text)
        {
            if (c >= font.getStartChar() && c <= font.getEndChar())
            {
                if (font.isFixedWidth())
                {
                    w += font.getWidth();
                }
                else
                {
                    w += font.getWidths()[c - font.getStartChar()];
                }
            }
        }
        return w;
    }

private:
    mxgui::Font font           = mxgui::tahoma;
    VertAlignment vert_align   = VertAlignment::TOP;
    HorizAlignment horiz_align = HorizAlignment::LEFT;
    std::string text;

    mxgui::Color col_text = mxgui::white;
    uint8_t padding;
};