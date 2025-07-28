/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Damiano Procaccia
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

/**
 * @file Numeric.h contains utility functions for numeric types, possibly
 * from newer C++ standards.
 */

#pragma once

#include <cstring>
#include <limits>
#include <type_traits>

namespace detail
{
/**
 * @brief Computes the saturating addition x + y for integral types.
 * @returns x + y, if the result is representable as a value of type T.
 * Otherwise, the largest or smallest value of type T, whichever is closer to
 * the result.
 */
template <class T>
auto add_sat(T x, T y) noexcept ->
    typename std::enable_if_t<std::is_integral<T>::value, T>
{
    auto result = T{};
    // The built-in function returns true if the addition overflows
    bool overflow = __builtin_add_overflow(x, y, &result);

    if (!overflow)
        return result;

    if (x < 0)
        return std::numeric_limits<T>::min();
    else
        return std::numeric_limits<T>::max();
}

/**
 * @brief Obtain a value of type To by reinterpreting the object representation
 * of From.
 *
 * This implementation is a polyfill for C++20's std::bit_cast.
 *
 * @returns A new object of type To with the same bit representation as src.
 */
template <typename To, typename From>
auto bit_cast(const From& src) noexcept ->
    typename std::enable_if_t<sizeof(To) == sizeof(From) &&
                                  std::is_trivially_copyable<To>::value &&
                                  std::is_trivially_copyable<From>::value,
                              To>
{
    static_assert(std::is_trivially_constructible<To>::value,
                  "This implementation additionally requires "
                  "destination type to be trivially constructible");

    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}

}  // namespace detail

namespace std
{
// C++20
#if __cplusplus < 202002L

using detail::bit_cast;

#endif

// C++26 (TODO: change to < 2026xxL when released)
#if __cplusplus <= 202302L

using detail::add_sat;

#endif

}  // namespace std
