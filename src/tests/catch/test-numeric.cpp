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

using namespace Boardcore;

TEST_CASE("Numeric - Saturating Add")
{
    constexpr int a = add_sat(3, 4);  // no saturation occurs, T = int
    REQUIRE(a == 7);

    constexpr unsigned char b =
        add_sat<unsigned char>(UCHAR_MAX, 4);  // saturated
    REQUIRE(b == UCHAR_MAX);

    constexpr unsigned char c = static_cast<unsigned char>(
        add_sat(UCHAR_MAX, 4));  // not saturated, T = int
                                 // add_sat(int, int) returns int tmp == 259,
                                 // then assignment truncates 259 % 256 == 3
    REQUIRE(c == 3);

    constexpr unsigned char e = add_sat<unsigned char>(251, a);  // saturated
    REQUIRE(e == UCHAR_MAX);
    // 251 is of type T = unsigned char, `a` is converted to unsigned char
    // value; might yield an int -> unsigned char conversion warning for `a`

    constexpr signed char f = add_sat<signed char>(-123, -3);  // not saturated
    REQUIRE(f == -126);

    constexpr signed char g = add_sat<signed char>(-123, -13);  // saturated
    REQUIRE(g == std::numeric_limits<signed char>::min());      // g == -128

    constexpr signed char h = add_sat<signed char>(126, CHAR_MAX);  // saturated
    REQUIRE(h == std::numeric_limits<signed char>::max());          // h == 127
}
