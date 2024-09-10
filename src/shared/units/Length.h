/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Basso
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
namespace Length
{

template <class Ratio = std::ratio<1>>
using Length = Unit<UnitKind::Length, Ratio>;

template <class ToLength, class FromLength>
ToLength length_cast(FromLength const &from)
{
    return ToLength(from);
}

using Millimeter = Length<std::milli>;  // Length in millimeters
using Centimeter = Length<std::centi>;  // Length in centimeters
using Decimeter  = Length<std::deci>;   // Length in decimeters
using Meter      = Length<>;            // Length in meters
using Kilometer  = Length<std::kilo>;   // Length in kilometers

// Floats
constexpr auto operator""_mm(long double n)
{
    return Millimeter(static_cast<float>(n));
};
constexpr auto operator""_cm(long double n)
{
    return Centimeter(static_cast<float>(n));
};
constexpr auto operator""_dm(long double n)
{
    return Decimeter(static_cast<float>(n));
};
constexpr auto operator""_m(long double n)
{
    return Meter(static_cast<float>(n));
};
constexpr auto operator""_km(long double n)
{
    return Kilometer(static_cast<float>(n));
};
// Integers
constexpr auto operator""_mm(unsigned long long n)
{
    return Millimeter(static_cast<float>(n));
};
constexpr auto operator""_cm(unsigned long long n)
{
    return Centimeter(static_cast<float>(n));
};
constexpr auto operator""_dm(unsigned long long n)
{
    return Decimeter(static_cast<float>(n));
};
constexpr auto operator""_m(unsigned long long n)
{
    return Meter(static_cast<float>(n));
};
constexpr auto operator""_km(unsigned long long n)
{
    return Kilometer(static_cast<float>(n));
};

}  // namespace Length
}  // namespace Units
}  // namespace Boardcore