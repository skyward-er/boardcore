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

#include <array>

namespace Boardcore
{

/**
 * @brief Implementation of a Median Filter
 *
 * @tparam windowSize The size of the window
 * @note WARNING: should be noted that the first windowSize - 1 outputs will be
 * wrong because the window is not full yet (consider using a Shadow Mode)
 */
template <size_t windowSize>
class MedFilter
{
    // windowSize must be odd
    static_assert(windowSize % 2 == 1, "windowSize must be odd");

public:
    /**
     * @brief Construct a new Median Filter object
     *
     * @note WARNING: Initialize output at 0 at first
     */
    explicit MedFilter() : window({}) {}

    /**
     * @brief Filter the input
     *
     * @param input The input to filter
     * @note WARNING: should be noted that the first windowSize - 1 outputs will
     * be wrong because the window is not full yet (consider using a Shadow
     * Mode)
     */
    float filter(float input)
    {
        slideWindow(input);
        return median();
    }

private:
    /**
     * @brief Slide the window by one position to the left and add the new input
     * to the right
     *
     * @param input The input to add to the window
     */
    void slideWindow(float input)
    {
        for (size_t i = 0; i < windowSize - 1; i++)
        {
            window[i] = window[i + 1];
        }
        window[windowSize - 1] = input;
    }

    /**
     * @brief Calculate the median of the window
     */
    float median()
    {
        std::array<float, windowSize> sortedWindow = window;
        std::sort(sortedWindow.begin(), sortedWindow.end());
        return sortedWindow[windowSize / 2];
    }

    std::array<float, windowSize> window;
};

}  // namespace Boardcore