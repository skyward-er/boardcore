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
namespace Pressure
{

template <class Ratio = std::ratio<1>>
using Pressure = Unit<UnitKind::Pressure, Ratio>;

template <class ToPressure, class FromPressure>
ToPressure pressure_cast(FromPressure const &from)
{
    return ToPressure(from);
}

using Pascal = Pressure<>;                    // Pressure in Pascals
using Bar    = Pressure<std::ratio<100000>>;  // Pressure in Bars
using Atm    = Pressure<std::ratio<101325>>;  // Pressure in Atmospheres

auto operator""_pa(long double n) { return Pascal(static_cast<float>(n)); };
auto operator""_bar(long double n) { return Bar(static_cast<float>(n)); };
auto operator""_atm(long double n) { return Atm(static_cast<float>(n)); };

}  // namespace Pressure
}  // namespace Units
}  // namespace Boardcore