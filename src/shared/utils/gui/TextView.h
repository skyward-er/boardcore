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

namespace Boardcore
{

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
     * @param textColor Text color
     * @param padding Spacing from view bounds in pixels
     */
    TextView(std::string text = "", mxgui::Color textColor = mxgui::white,
             uint8_t padding = 1)
        : View(), text(text), colText(textColor), padding(padding)
    {
    }

    void setVerticalAlignment(VertAlignment alignment)
    {
        vertAlign = alignment;
        invalidate();
    }

    void setHorizontalAlignment(HorizAlignment alignment)
    {
        horizAlign = alignment;
        invalidate();
    }

    void setAlignment(HorizAlignment horizAlign, VertAlignment vertAlign)
    {
        setVerticalAlignment(vertAlign);
        setHorizontalAlignment(horizAlign);
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
        if (color != colText)
        {
            this->colText = color;
            invalidate();
        }
    }

    mxgui::Color getTextColor() { return colText; }

    virtual void draw(mxgui::DrawingContext& dc) override
    {
        if (isInvalidated())
        {
            View::draw(dc);

            Bounds paddedBounds = getBounds();
            paddedBounds.pos += Position{padding, padding};
            paddedBounds.size.height -= padding;
            paddedBounds.size.width -= padding;

            Position topLeft = paddedBounds.topLeft();
            switch (vertAlign)
            {
                case VertAlignment::BOTTOM:
                    topLeft.y =
                        paddedBounds.bottomLeft().y - dc.getFont().getHeight();
                    break;
                case VertAlignment::CENTER:
                    topLeft.y =
                        paddedBounds.pos.y +
                        (paddedBounds.size.height - dc.getFont().getHeight()) /
                            2;
                    break;
                case VertAlignment::TOP:
                    break;
            }

            switch (horizAlign)
            {
                case HorizAlignment::RIGHT:
                    topLeft.x = paddedBounds.topRight().x - getTextWidth(text);
                    break;
                case HorizAlignment::CENTER:
                    topLeft.x =
                        paddedBounds.pos.x +
                        (paddedBounds.size.width - getTextWidth(text)) / 2;
                    break;
                case HorizAlignment::LEFT:
                    break;
            }

            mxgui::Font oldFont = dc.getFont();
            dc.setFont(font);

            dc.setTextColor(getTextColor(), getBackgroundColor());
            dc.clippedWrite(topLeft, paddedBounds.topLeft(),
                            paddedBounds.bottomRight(), text);

            dc.setFont(oldFont);
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
    mxgui::Font font          = mxgui::tahoma;
    VertAlignment vertAlign   = VertAlignment::TOP;
    HorizAlignment horizAlign = HorizAlignment::LEFT;
    std::string text;

    mxgui::Color colText = mxgui::white;
    uint8_t padding;
};

}  // namespace Boardcore
