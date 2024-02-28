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

namespace Boardcore
{
namespace Constants
{
constexpr long long NS_IN_MS = 1000000ll;     // Nanoseconds in 1 ms
constexpr long long NS_IN_S  = 1000000000ll;  // Nanoseconds in 1 s
}  // namespace Constants

/**
 * @brief Convert nanoseconds to milliseconds.
 *
 * @param ns Nanoseconds.
 * @return Milliseconds.
 */
constexpr long long nsToMs(long long ns) { return ns / Constants::NS_IN_MS; }

/**
 * @brief Convert milliseconds to nanoseconds.
 *
 * @param ms Milliseconds.
 * @return Nanoseconds.
 */
constexpr long long msToNs(long long ms) { return ms * Constants::NS_IN_MS; }

/**
 * @brief Convert seconds to nanoseconds.
 *
 * @param s Seconds.
 * @return Nanoseconds.
 */
constexpr long long sToNs(long long s) { return s * Constants::NS_IN_S; }

}  // namespace Boardcore
