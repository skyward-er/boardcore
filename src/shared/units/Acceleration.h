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

#include <utils/Constants.h>

#include <ratio>

#include "Units.h"

namespace Boardcore
{
namespace Units
{
namespace Acceleration
{

template <class Ratio = std::ratio<1>>
using Acceleration = Unit<UnitKind::Acceleration, Ratio>;

template <class ToAcceleration, class FromAcceleration>
ToAcceleration acceleration_cast(FromAcceleration const& from)
{
    return ToAcceleration(from);
}

using MeterPerSecondSquared = Acceleration<>;  // Acceleration in m/s^2
using G =
    Acceleration<std::ratio<static_cast<std::intmax_t>(Constants::g * 1e10),
                            static_cast<std::intmax_t>(1e10)>>;  // Acceleration
                                                                 // in Gs

/**
 * Assert that we have enough precision to represent the constant value.
 */
static_assert(MeterPerSecondSquared{G{1}}.value() == Constants::g,
              "Not enough precision to represent g in m/s^2");

// Floats
constexpr auto operator""_mps2(long double n)
{
    return MeterPerSecondSquared(static_cast<float>(n));
};
constexpr auto operator""_g(long double n) { return G(static_cast<float>(n)); };
// Integers
constexpr auto operator""_mps2(unsigned long long n)
{
    return MeterPerSecondSquared(static_cast<float>(n));
};
constexpr auto operator""_g(unsigned long long n)
{
    return G(static_cast<float>(n));
};

}  // namespace Acceleration
}  // namespace Units
}  // namespace Boardcore
