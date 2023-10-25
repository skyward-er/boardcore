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

template <typename T, size_t D>
class SlidingWindow
{
public:
    explicit SlidingWindow() : window({}), filled(0) {}

    void push(T value)
    {
        shiftWindow(1);
        filled < D ? filled++ : filled;
        setLast(value);
    }

    bool isFull() { return filled == D; }

    T last() { return window[D - 1]; }

    std::array<T, D>& all() { return &window; }

private:
    inline void setLast(T value) { window[D - 1] = value; }

    /**
     * @brief Shift the window by `n` positions to the left leaving the last
     * values unchanged (hard assumption, be sure to know what you're doing)
     *
     * @param input The input to add to the window
     */
    void shiftWindow(size_t n)
    {
        for (int i = 0; i < D - n; i++)
        {
            window[i] = window[i + n];
        }
    }

    std::array<T, D> window;
    size_t filled;
};

}  // namespace Boardcore
