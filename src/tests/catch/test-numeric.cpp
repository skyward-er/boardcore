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

#include <utils/Numeric.h>

#include <catch2/catch.hpp>
#include <climits>

TEST_CASE("Numeric - std::add_sat polyfill")
{
    const int a = std::add_sat(3, 4);  // no saturation occurs, T = int
    REQUIRE(a == 7);

    const unsigned char b =
        std::add_sat<unsigned char>(UCHAR_MAX, 4);  // saturated
    REQUIRE(b == UCHAR_MAX);

    const unsigned char c = static_cast<unsigned char>(std::add_sat(
        UCHAR_MAX, 4));  // not saturated, T = int
                         // std::add_sat(int, int) returns int tmp == 259,
                         // then assignment truncates 259 % 256 == 3
    REQUIRE(c == 3);

    const unsigned char e = std::add_sat<unsigned char>(251, a);  // saturated
    REQUIRE(e == UCHAR_MAX);
    // 251 is of type T = unsigned char, `a` is converted to unsigned char
    // value; might yield an int -> unsigned char conversion warning for `a`

    const signed char f = std::add_sat<signed char>(-123, -3);  // not saturated
    REQUIRE(f == -126);

    const signed char g = std::add_sat<signed char>(-123, -13);  // saturated
    REQUIRE(g == std::numeric_limits<signed char>::min());       // g == -128

    const signed char h =
        std::add_sat<signed char>(126, CHAR_MAX);           // saturated
    REQUIRE(h == std::numeric_limits<signed char>::max());  // h == 127
}

TEST_CASE("Numeric - std::bit_cast polyfill")
{
    const double f64v = 19880124.0;
    const auto u64v   = std::bit_cast<std::uint64_t>(f64v);
    REQUIRE(std::bit_cast<double>(u64v) == f64v);  // round-trip

    const std::uint64_t u64v2 = 0x3fe9000000000000ull;
    const auto f64v2          = std::bit_cast<double>(u64v2);
    REQUIRE(std::bit_cast<std::uint64_t>(f64v2) == u64v2);  // round-trip
}
