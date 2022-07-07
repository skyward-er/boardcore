/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

namespace Boardcore
{

struct AirBrakesConfig
{
    // Coefficient of drag coefficients.
    float N000;
    float N100;
    float N200;
    float N300;
    float N400;
    float N500;
    float N600;
    float N010;
    float N020;
    float N110;
    float N120;
    float N210;
    float N220;
    float N310;
    float N320;
    float N410;
    float N420;
    float N510;
    float N520;
    float N001;

    // Aibrakes extension.
    float EXTENSION;  ///< [m]
    uint8_t DRAG_STEPS;
    float EXT_POL_1;
    float EXT_POL_2;
    float EXT_POL_3;
    float EXT_POL_4;

    float S0;       ///< Rocket surface [m^2]
    float SURFACE;  ///< AirBrakes max surface [m^2]

    // PI parameters
    float KP;
    float KI;
    float TS;
};

}  // namespace Boardcore
