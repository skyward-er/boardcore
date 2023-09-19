/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <vector>

namespace Boardcore
{

// ------------------------ INPUT ------------------------
// result of sin(50 * 2πt) + 2sin(25 * 2πt) with t sampling time
// that sums to 1 period (64 samples)
static const std::vector<float> SIGNAL{
    0.000000,   0.288001,    -2.344254,  2.595312,   -0.058260, -1.136140,
    0.739060,   -1.795279,   2.414214,   -0.000944,  -2.035020, 1.358310,
    -1.140652,  1.774263,    0.007497,   -2.526806,  2.000000,  -0.565236,
    0.772864,   0.111324,    -2.554866,  2.469451,   -0.187261, -0.391125,
    0.414214,   -2.185460,   2.586819,   -0.024999,  -1.472474, 0.932373,
    -1.578887,  2.249572,    0.000000,   -2.249572,  1.578887,  -0.932373,
    1.472474,   0.024999,    -2.586819,  2.185460,   -0.414214, 0.391125,
    0.187261,   -2.469451,   2.554866,   -0.111324,  -0.772864, 0.565236,
    -2.000000,  2.526806,    -0.007497,  -1.774263,  1.140652,  -1.358310,
    2.035020,   0.000944,    -2.414214,  1.795279,   -0.739060, 1.136140,
    0.05826008, -2.59531214, 2.34425399, -0.28800129};

static const int SAMPLES       = 64;
static const float SAMPLE_RATE = 64.0;

// ------------------------ EXPECTED OUTPUT ------------------------
static const std::vector<float> INTENSITY{
    0., 0., 0.,  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.5, 0.,
    0., 0., 0.,  0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0.,  0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,  0.,
    0., 0., 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,  0.};

static const std::vector<float> FREQUENCY{
    0.0,   1.0,   2.0,   3.0,   4.0,   5.0,   6.0,   7.0,   8.0,   9.0,   10.0,
    11.0,  12.0,  13.0,  14.0,  15.0,  16.0,  17.0,  18.0,  19.0,  20.0,  21.0,
    22.0,  23.0,  24.0,  25.0,  26.0,  27.0,  28.0,  29.0,  30.0,  31.0,  -32.0,
    -31.0, -30.0, -29.0, -28.0, -27.0, -26.0, -25.0, -24.0, -23.0, -22.0, -21.0,
    -20.0, -19.0, -18.0, -17.0, -16.0, -15.0, -14.0, -13.0, -12.0, -11.0, -10.0,
    -9.0,  -8.0,  -7.0,  -6.0,  -5.0,  -4.0,  -3.0,  -2.0,  -1.0};

}  // namespace Boardcore
