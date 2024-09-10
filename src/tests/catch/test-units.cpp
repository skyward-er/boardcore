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

#include <units/Acceleration.h>
#include <units/Angle.h>
#include <units/Length.h>
#include <units/Pressure.h>
#include <units/Speed.h>
#include <units/Time.h>
#include <utils/Debug.h>

#include <catch2/catch.hpp>
#include <cmath>

using namespace Boardcore;

template <class T>
bool eq(T const &a, T const &b)
{
    return std::fabs(a.value() - b.value()) < 1e-6;
}

TEST_CASE("Units Test")
{
    constexpr float PI = 3.14159265358979323846;

    using namespace Units::Angle;
    using namespace Units::Length;
    using namespace Units::Pressure;
    using namespace Units::Time;
    using namespace Units::Speed;
    using namespace Units::Acceleration;

    Meter len = 1_m;

    // Verify ratios
    REQUIRE(Radian(PI) == angle_cast<Radian>(Degree(180)));

    REQUIRE(Millimeter(1000) == length_cast<Millimeter>(Meter(1)));
    REQUIRE(Centimeter(100) == length_cast<Centimeter>(Meter(1)));
    REQUIRE(Decimeter(10) == length_cast<Decimeter>(Meter(1)));
    REQUIRE(Kilometer(0.001) == length_cast<Kilometer>(Meter(1)));

    REQUIRE(Pascal(100000) == pressure_cast<Pascal, Bar>(Bar(1)));
    REQUIRE(Atm(1) == pressure_cast<Atm, Pascal>(Pascal(101325)));

    REQUIRE(Nanosecond(1000000000) == time_cast<Nanosecond>(Second(1)));
    REQUIRE(Microsecond(1000000) == time_cast<Microsecond>(Second(1)));
    REQUIRE(Millisecond(1000) == time_cast<Millisecond>(Second(1)));
    REQUIRE(Minute(1) == time_cast<Minute>(Second(60)));
    REQUIRE(Hour(1) == time_cast<Hour>(Second(3600)));

    REQUIRE(MeterPerSecond(1) ==
            speed_cast<MeterPerSecond>(KilometerPerHour(3.6)));

    REQUIRE(MeterPerSecondSquared(9.81) ==
            acceleration_cast<MeterPerSecondSquared>(G(1)));

    // Test operators
    REQUIRE(eq(Radian(2 * PI), Radian(PI) + Radian(PI)));
    REQUIRE(eq(Radian(0), Radian(PI) - Radian(PI)));
    REQUIRE(eq(Radian(PI * 2), Radian(PI) * 2));
    REQUIRE(eq(Radian(2 * PI), 2 * Radian(PI)));
    REQUIRE(eq(Radian(PI / 2), Radian(PI) / 2));
    REQUIRE(eq(Radian(2 / PI), 2 / Radian(PI)));

    // Test comparisons
    REQUIRE(Radian(PI) == Radian(PI));
    REQUIRE(Radian(PI) != Radian(2 * PI));
    REQUIRE(Radian(PI) < Radian(2 * PI));
    REQUIRE(Radian(PI) <= Radian(2 * PI));
    REQUIRE(Radian(PI) <= Radian(PI));
    REQUIRE(Radian(2 * PI) > Radian(PI));
    REQUIRE(Radian(2 * PI) >= Radian(PI));
    REQUIRE(Radian(PI) >= Radian(PI));

    // Test assignment operators
    auto a = Radian(PI);
    a += Radian(PI);
    REQUIRE(a == Radian(2 * PI));
    a -= Radian(PI);
    REQUIRE(a == Radian(PI));
    a *= 2;
    REQUIRE(a == Radian(2 * PI));
    a /= 2;
    REQUIRE(a == Radian(PI));
}