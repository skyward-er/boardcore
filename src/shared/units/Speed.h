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
namespace Speed
{

template <class Ratio = std::ratio<1>>
using Speed = Unit<UnitKind::Speed, Ratio>;

template <class ToSpeed, class FromSpeed>
ToSpeed speed_cast(FromSpeed const &from)
{
    return ToSpeed(from);
}

using MeterPerSecond = Speed<>;  // Speed in meters per second
using KilometerPerHour =
    Speed<std::ratio<1000, 3600>>;  // Speed in kilometers per hour

// Floats
constexpr auto operator""_mps(long double n)
{
    return MeterPerSecond(static_cast<float>(n));
};
constexpr auto operator""_kmh(long double n)
{
    return KilometerPerHour(static_cast<float>(n));
};
// Integers
constexpr auto operator""_mps(unsigned long long n)
{
    return MeterPerSecond(static_cast<float>(n));
};
constexpr auto operator""_kmh(unsigned long long n)
{
    return KilometerPerHour(static_cast<float>(n));
};

}  // namespace Speed
}  // namespace Units
}  // namespace Boardcore