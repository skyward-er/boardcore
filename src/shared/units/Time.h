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
class Time : public Unit<Ratio>
{
    using Unit<Ratio>::Unit;

public:
    std::chrono::duration<float> chrono() const
    {
        return std::chrono::duration<float, Ratio>(this->value());
    }
};

template <class ToTime, class FromTime>
ToTime time_cast(FromTime &from)
{
    return ToTime(from);
}

using Nanosecond  = Time<std::ratio<1, 1000000000>>;  // Time in nanoseconds
using Microsecond = Time<std::ratio<1, 1000000>>;     // Time in microseconds
using Millisecond = Time<std::ratio<1, 1000>>;        // Time in milliseconds
using Second      = Time<>;                           // Time in seconds
using Minute      = Time<std::ratio<60>>;             // Time in minutes
using Hour        = Time<std::ratio<3600>>;           // Time in hours
using Day         = Time<std::ratio<86400>>;          // Time in days
using Week        = Time<std::ratio<604800>>;         // Time in weeks
using Month       = Time<std::ratio<262800>>;         // Time in months
using Year        = Time<std::ratio<31536000>>;       // Time in years

auto operator""_ns(long double n) { return Nanosecond(static_cast<float>(n)); };
auto operator""_us(long double n)
{
    return Microsecond(static_cast<float>(n));
};
auto operator""_ms(long double n)
{
    return Millisecond(static_cast<float>(n));
};
auto operator""_s(long double n) { return Second(static_cast<float>(n)); };
auto operator""_min(long double n) { return Minute(static_cast<float>(n)); };
auto operator""_h(long double n) { return Hour(static_cast<float>(n)); };
auto operator""_d(long double n) { return Day(static_cast<float>(n)); };
auto operator""_w(long double n) { return Week(static_cast<float>(n)); };
auto operator""_mo(long double n) { return Month(static_cast<float>(n)); };
auto operator""_y(long double n) { return Year(static_cast<float>(n)); };

}  // namespace Time
}  // namespace Units
}  // namespace Boardcore