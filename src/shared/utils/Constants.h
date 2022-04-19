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

namespace Constants
{

static constexpr const float PI                 = 3.14159265f;  // [rad]
static constexpr const float DEGREES_TO_RADIANS = PI / 180.0f;
static constexpr const float RADIANS_TO_DEGREES = 180.0f / PI;

static constexpr const float g = 9.80665f;  // [m^s^2]

constexpr float a    = 0.0065f;  // Troposphere temperature gradient [deg/m]
constexpr float R    = 287.05f;  // Air gas constant [J/Kg/K]
constexpr float n    = g / (R * a);
constexpr float nInv = (R * a) / g;

}  // namespace Constants

}  // namespace Boardcore
