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

#include <utils/Debug.h>

#include <ratio>
#include <typeinfo>

namespace Boardcore
{
namespace Units
{

enum class UnitKind
{
    Angle,
    Length,
    Pressure,
    Time,
    Speed,
    Acceleration,
};

// Base class to implement custom measurement units logic.
template <UnitKind Kind, class Ratio = std::ratio<1>>
class Unit
{
public:
    constexpr explicit Unit(float val) : _value(val) {}

    template <UnitKind FromKind, class FromRatio>
    constexpr explicit Unit(Unit<FromKind, FromRatio> const &from)
        : _value(from.template value<Ratio>())
    {
    }

    // Get the value of the unit in the specified ratio.
    template <class TargetRatio = Ratio>
    constexpr float value() const
    {
        constexpr auto currentRatio =
            static_cast<float>(Ratio::num) / static_cast<float>(Ratio::den);
        constexpr auto targetRatio = static_cast<float>(TargetRatio::num) /
                                     static_cast<float>(TargetRatio::den);

        return _value * currentRatio / targetRatio;
    }

    template <UnitKind TargetKind, class TargetRatio = Ratio>
    constexpr explicit operator Unit<TargetKind, TargetRatio>() const
    {
        return Unit<TargetKind, TargetRatio>(value<TargetRatio>());
    }

private:
    float _value;
};

// Sum, Subtraction, Multiplication, Division
template <UnitKind Kind, class Ratio>
constexpr auto operator+(const Unit<Kind, Ratio> &lhs,
                         const Unit<Kind, Ratio> &rhs)
{
    return Unit<Kind, Ratio>(lhs.template value() + rhs.template value());
}

template <UnitKind Kind, class Ratio>
constexpr auto operator-(const Unit<Kind, Ratio> &lhs,
                         const Unit<Kind, Ratio> &rhs)
{
    return Unit<Kind, Ratio>(lhs.template value() - rhs.template value());
}

template <UnitKind Kind, class Ratio>
constexpr auto operator*(const Unit<Kind, Ratio> &lhs, float rhs)
{
    return Unit<Kind, Ratio>(lhs.template value() * rhs);
}

template <UnitKind Kind, class Ratio>
constexpr auto operator*(float lhs, const Unit<Kind, Ratio> &rhs)
{
    return Unit<Kind, Ratio>(lhs * rhs.template value());
}

template <UnitKind Kind, class Ratio>
constexpr auto operator/(const Unit<Kind, Ratio> &lhs, float rhs)
{
    return Unit<Kind, Ratio>(lhs.template value() / rhs);
}

template <UnitKind Kind, class Ratio>
constexpr auto operator/(float lhs, const Unit<Kind, Ratio> &rhs)
{
    return Unit<Kind, Ratio>(lhs / rhs.template value());
}

// Comparison operators
template <UnitKind Kind, class Ratio>
constexpr bool operator==(const Unit<Kind, Ratio> &lhs,
                          const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() == rhs.template value();
}

template <UnitKind Kind, class Ratio>
constexpr bool operator!=(const Unit<Kind, Ratio> &lhs,
                          const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() != rhs.template value();
}

template <UnitKind Kind, class Ratio>
constexpr bool operator<(const Unit<Kind, Ratio> &lhs,
                         const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() < rhs.template value();
}

template <UnitKind Kind, class Ratio>
constexpr bool operator>(const Unit<Kind, Ratio> &lhs,
                         const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() > rhs.template value();
}

template <UnitKind Kind, class Ratio>
constexpr bool operator<=(const Unit<Kind, Ratio> &lhs,
                          const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() <= rhs.template value();
}

template <UnitKind Kind, class Ratio>
constexpr bool operator>=(const Unit<Kind, Ratio> &lhs,
                          const Unit<Kind, Ratio> &rhs)
{
    return lhs.template value() >= rhs.template value();
}

// Direct assignment operators
template <UnitKind Kind, class Ratio>
constexpr Unit<Kind, Ratio> &operator+=(Unit<Kind, Ratio> &lhs,
                                        const Unit<Kind, Ratio> &rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

template <UnitKind Kind, class Ratio>
constexpr Unit<Kind, Ratio> &operator-=(Unit<Kind, Ratio> &lhs,
                                        const Unit<Kind, Ratio> &rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

template <UnitKind Kind, class Ratio>
constexpr Unit<Kind, Ratio> &operator*=(Unit<Kind, Ratio> &lhs, float rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

template <UnitKind Kind, class Ratio>
constexpr Unit<Kind, Ratio> &operator/=(Unit<Kind, Ratio> &lhs, float rhs)
{
    lhs = lhs / rhs;
    return lhs;
}

}  // namespace Units
}  // namespace Boardcore
