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
    Length
};

template <UnitKind Kind, class Ratio = std::ratio<1>>
// Base class to implement custom measurement units logic.
class Unit
{
public:
    Unit(float val) : _value(val){};
    template <class FromRatio>
    constexpr explicit Unit(Unit<Kind, FromRatio> const &from)
        : _value(from.template value<Ratio>())
    {
    }

    // Get the value of the unit in the specified ratio.
    template <class TargetRatio = Ratio>
    float value() const
    {
        const auto currentRatio =
            static_cast<float>(Ratio::num) / static_cast<float>(Ratio::den);
        const auto targetRatio = static_cast<float>(TargetRatio::num) /
                                 static_cast<float>(TargetRatio::den);

        return _value * currentRatio / targetRatio;
    }

    template <class TargetRatio = Ratio>
    constexpr explicit operator Unit<TargetRatio>() const
    {
        return Unit<TargetRatio>(value<TargetRatio>());
    }

private:
    float _value;
};

// Sum, Subtraction, Multiplication, Division
template <UnitKind Kind, class Ratio>
constexpr auto operator+(const Unit<Kind, Ratio> &lhs,
                         const Unit<Kind, Ratio> &rhs)
{
    return Unit(lhs.template value() + rhs.template value());
}

template <class DerivedUnit>
constexpr auto operator-(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return DerivedUnit(lhs.template value() - rhs.template value());
}

template <class DerivedUnit>
constexpr auto operator*(const DerivedUnit &lhs, float rhs)
{
    return DerivedUnit(lhs.template value() * rhs);
}

template <class DerivedUnit>
constexpr auto operator*(float lhs, const DerivedUnit &rhs)
{
    return DerivedUnit(lhs * rhs.template value());
}

template <class DerivedUnit>
constexpr auto operator/(const DerivedUnit &lhs, float rhs)
{
    return DerivedUnit(lhs.template value() / rhs);
}

template <class DerivedUnit>
constexpr auto operator/(float lhs, const DerivedUnit &rhs)
{
    return DerivedUnit(lhs / rhs.template value());
}

// Comparison operators
template <class DerivedUnit>
constexpr bool operator==(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() == rhs.template value();
}

template <class DerivedUnit>
constexpr bool operator!=(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() != rhs.template value();
}

template <class DerivedUnit>
constexpr bool operator<(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() < rhs.template value();
}

template <class DerivedUnit>
constexpr bool operator>(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() > rhs.template value();
}

template <class DerivedUnit>
constexpr bool operator<=(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() <= rhs.template value();
}

template <class DerivedUnit>
constexpr bool operator>=(const DerivedUnit &lhs, const DerivedUnit &rhs)
{
    return lhs.template value() >= rhs.template value();
}

// Direct assignment operators
template <class DerivedUnit>
constexpr DerivedUnit &operator+=(DerivedUnit &lhs, const DerivedUnit &rhs)
{
    lhs = lhs + rhs;
    return lhs;
}

template <class DerivedUnit>
constexpr DerivedUnit &operator-=(DerivedUnit &lhs, const DerivedUnit &rhs)
{
    lhs = lhs - rhs;
    return lhs;
}

template <class DerivedUnit>
constexpr DerivedUnit &operator*=(DerivedUnit &lhs, float rhs)
{
    lhs = lhs * rhs;
    return lhs;
}

template <class DerivedUnit>
constexpr DerivedUnit &operator/=(DerivedUnit &lhs, float rhs)
{
    lhs = lhs / rhs;
    return lhs;
}

}  // namespace Units
}  // namespace Boardcore
