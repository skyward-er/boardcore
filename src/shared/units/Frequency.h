/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <cstdint>

#include "Units.h"

namespace Boardcore
{

namespace Units
{

namespace Frequency
{

template <class Ratio = std::ratio<1>>
using Frequency = Unit<UnitKind::Frequency, Ratio>;

using Hertz     = Frequency<>;
using Kilohertz = Frequency<std::kilo>;

// Integers
constexpr auto operator""_hz(unsigned long long n)
{
    return Hertz(static_cast<float>(n));
};

constexpr auto operator""_khz(unsigned long long n)
{
    return Kilohertz(static_cast<float>(n));
};

// Floats
constexpr auto operator""_hz(long double n)
{
    return Hertz(static_cast<float>(n));
};

constexpr auto operator""_khz(long double n)
{
    return Kilohertz(static_cast<float>(n));
};

}  // namespace Frequency

}  // namespace Units

}  // namespace Boardcore
