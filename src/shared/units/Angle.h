/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Davide Basso
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

#include <ratio>

#include "Units.h"

namespace Boardcore
{
namespace Units
{
namespace Angle
{

template <class Ratio = std::ratio<1>>
using Angle = Unit<UnitKind::Angle, Ratio>;

using Degree = Angle<>;  // Angle in degrees
using Radian =           // Angle in radians
    Angle<
        std::ratio<static_cast<std::intmax_t>(180 * 1e10),
                   static_cast<std::intmax_t>(3.14159265358979323846 * 1e10)>>;

auto operator""_rad(long double n) { return Radian(static_cast<float>(n)); };
auto operator""_deg(long double n) { return Degree(static_cast<float>(n)); };

template <class ToAngle, class FromAngle>
ToAngle angle_cast(FromAngle const &from)
{
    return ToAngle(from);
}

}  // namespace Angle
}  // namespace Units
}  // namespace Boardcore