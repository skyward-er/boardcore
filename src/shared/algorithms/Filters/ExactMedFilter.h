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
 * @brief Exact Median Filter based on a fixed window size, mind that while it
 * is not full the filtered output should not be taken into consideration
 *
 * @tparam WIN_SIZE The exact size of the window
 * @warning: should be noted that the first WIN_SIZE - 1 outputs will be
 * wrong because the window is not full yet (consider using a Shadow Mode)
 */
template <size_t WIN_SIZE>
class ExactMedFilter
{
    // WIN_SIZE must be odd
    static_assert(WIN_SIZE % 2 == 1, "WIN_SIZE must be odd");

public:
    /**
     * @brief Construct a new Exact Median Filter object
     *
     * @warning Initialize output at 0 at first
     */
    explicit ExactMedFilter() : window() {}

    /**
     * @brief Filter the input
     *
     * @param input The input to filter
     * @warning should be noted that the first WIN_SIZE - 1 outputs will
     * be wrong because the window is not full yet (consider using a Shadow
     * Mode)
     */
    float filter(float input)
    {
        window.push(input);
        return median();
    }

private:
    /**
     * @brief Calculate the median of the window
     */
    float median()
    {
        std::array<float, WIN_SIZE> sorted_window = window.all();
        std::sort(sorted_window.begin(), sorted_window.end());
        return sorted_window[WIN_SIZE / 2];
    }

    SlidingWindow<float, WIN_SIZE> window;
};

}  // namespace Boardcore
