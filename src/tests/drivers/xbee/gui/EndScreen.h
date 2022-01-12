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
#include <utils/gui/GridLayout.h>
#include <utils/gui/OptionView.h>
#include <utils/gui/TextView.h>
#include <utils/gui/VerticalLayout.h>

#include <cstdint>

namespace Boardcore
{

struct EndScreen
{
    EndScreen()
    {
        tvTitle.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        tvF.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        tvReset.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tvF.setSelectable(true);
        tvReset.setSelectable(true);

        root.addView(&tvTitle, 1);
        root.addView(&tvF, 1);
        root.addView(&tvReset, 1);
    }

    VerticalLayout root;

    TextView tvF{"Press F to pay respects"};
    TextView tvReset{"Or RESET to start again"};

private:
    TextView tvTitle{"You killed the program, you maniac!"};
};

}  // namespace Boardcore
