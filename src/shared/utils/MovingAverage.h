/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

template <typename T>
class MovingAverage
{
public:
    MovingAverage(int movingAverageN = 10) : movingAverageN(movingAverageN) {}

    T getValue() { return value; }

    void updateValue(T newValue)
    {
        if (value == 0)
        {
            value = newValue;
            return;
        }

        value *= movingAverageCompCoeff;
        value += newValue * movingAverageCoeff;
    }

    void reset() { value = 0; }

    void setN(int movingAverageN)
    {
        this->movingAverageN   = movingAverageN;
        movingAverageCoeff     = 1 / (float)movingAverageN;
        movingAverageCompCoeff = 1 - movingAverageCoeff;
    }

private:
    T value = 0;

    int movingAverageN           = 20;
    float movingAverageCoeff     = 1 / (float)movingAverageN;
    float movingAverageCompCoeff = 1 - movingAverageCoeff;
};

}  // namespace Boardcore
