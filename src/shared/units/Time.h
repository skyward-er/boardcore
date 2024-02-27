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

#include <chrono>
#include <ratio>

#include "Units.h"

namespace Boardcore
{
namespace Units
{
namespace Time
{

template <class Ratio = std::ratio<1>>
using Time = Unit<UnitKind::Time, Ratio>;

template <class ToTime, class FromTime>
ToTime time_cast(FromTime const &from)
{
    return ToTime(from);
}

std::chrono::duration<float> to_chrono(Time<> const &from)
{
    return std::chrono::duration<float>(from.value());
}

using Nanosecond  = Time<std::ratio<1, 1000000000>>;  // Time in nanoseconds
using Microsecond = Time<std::ratio<1, 1000000>>;     // Time in microseconds
using Millisecond = Time<std::ratio<1, 1000>>;        // Time in milliseconds
using Second      = Time<>;                           // Time in seconds
using Minute      = Time<std::ratio<60>>;             // Time in minutes
using Hour        = Time<std::ratio<3600>>;           // Time in hours

// Floats
constexpr auto operator""_ns(long double n)
{
    return Nanosecond(static_cast<float>(n));
};
constexpr auto operator""_us(long double n)
{
    return Microsecond(static_cast<float>(n));
};
constexpr auto operator""_ms(long double n)
{
    return Millisecond(static_cast<float>(n));
};
constexpr auto operator""_s(long double n)
{
    return Second(static_cast<float>(n));
};
constexpr auto operator""_min(long double n)
{
    return Minute(static_cast<float>(n));
};
constexpr auto operator""_h(long double n)
{
    return Hour(static_cast<float>(n));
};
// Integers
constexpr auto operator""_ns(unsigned long long n)
{
    return Nanosecond(static_cast<float>(n));
};
constexpr auto operator""_us(unsigned long long n)
{
    return Microsecond(static_cast<float>(n));
};
constexpr auto operator""_ms(unsigned long long n)
{
    return Millisecond(static_cast<float>(n));
};
constexpr auto operator""_s(unsigned long long n)
{
    return Second(static_cast<float>(n));
};
constexpr auto operator""_min(unsigned long long n)
{
    return Minute(static_cast<float>(n));
};
constexpr auto operator""_h(unsigned long long n)
{
    return Hour(static_cast<float>(n));
};

}  // namespace Time
}  // namespace Units
}  // namespace Boardcore