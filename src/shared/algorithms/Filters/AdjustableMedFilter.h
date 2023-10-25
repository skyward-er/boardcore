/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include <utils/SlidingWindow.h>

#include <array>

namespace Boardcore
{

/**
 * @brief Median Filter with adjustable window size in order to use the widest
 * window size available if enough values have been pushed through
 *
 * @tparam WIN_SIZE The maximum size of the window
 */
template <size_t WIN_SIZE>
class AdjustableMedFilter
{
    // WIN_SIZE must be odd
    static_assert(WIN_SIZE % 2 == 1, "WIN_SIZE must be odd");

public:
    explicit AdjustableMedFilter() : window() {}

    float filter(float input)
    {
        window.push(input);
        return median();
    }

private:
    /**
     * @warning assumption: UB if the window completely empty
     */
    float median()
    {
        std::array<float, WIN_SIZE> sorted_window = window.all();

        size_t fil = window.filled();
        // reduce to the nearest odd number
        size_t best_win_size = fil % 2 && fil > 0 ? fil - 1 : fil;

        std::sort(sorted_window.begin() + WIN_SIZE - best_win_size,
                  sorted_window.end());

        return sorted_window[best_win_size / 2 + WIN_SIZE - best_win_size];
    }

    SlidingWindow<float, WIN_SIZE> window;
};

}  // namespace Boardcore
