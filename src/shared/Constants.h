/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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

static constexpr const float PI                 = 3.14159265f;
static constexpr const float EARTH_GRAVITY      = 9.80665f;
static constexpr const float EARTH_RADIUS       = 6371.0f * 1000.0f;  // [m]
static constexpr const float DEGREES_TO_RADIANS = PI / 180.0f;
static constexpr const float RADIANS_TO_DEGREES = 180.0f / PI;
static constexpr const float KNOTS_TO_MPS       = 0.514444;
static constexpr const float MSL_PRESSURE       = 101325.0f;  // [Pa]
static constexpr const float MSL_TEMPERATURE    = 288.15f;    // [Kelvin]

}  // namespace Boardcore
