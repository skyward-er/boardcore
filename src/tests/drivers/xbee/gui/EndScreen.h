/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <cstdint>
#include <mxgui/display.h>

#include "utils/gui/GridLayout.h"
#include "utils/gui/TextView.h"
#include "utils/gui/VerticalLayout.h"
#include "utils/gui/OptionView.h"



struct EndScreen
{
    EndScreen()
    {
        tv_title.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        tv_f.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);
        tv_reset.setAlignment(HorizAlignment::CENTER, VertAlignment::CENTER);

        tv_f.setSelectable(true);
        tv_reset.setSelectable(true);

        root.addView(&tv_title, 1);
        root.addView(&tv_f, 1);
        root.addView(&tv_reset, 1);
    }

    VerticalLayout root;

    TextView tv_f{"Press F to pay respects"};
    TextView tv_reset{"Or RESET to start again"};
private:
    TextView tv_title{"You killed the program, you maniac!"}; 
};