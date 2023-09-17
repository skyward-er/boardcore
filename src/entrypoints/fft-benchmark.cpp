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

#include <iostream>
#include <Eigen/Dense>
#include <miosix.h>
#include <complex.h>
#include <math.h>
#include <algorithms/FFT.h>

using namespace Boardcore;

int main(int argc, char const *argv[])
{
    Eigen::Vector<float, 256> input_signal = Eigen::Vector<float, 256>::Zero();
    float f1, f2;

    f1 = 50;
    f2 = 25;

    for (size_t i = 0; i < input_signal.size(); i++)
    {
        input_signal(i) = sin(2 * M_PI * i / 256 * f1) + sin(2 * M_PI * i / 256 * f2);
    }

    Eigen::Vector<std::complex<float>, 256> fft_result = FFT<256>::fft(input_signal);
    Eigen::Vector<float, 256> fft_freq = FFT<256>::fftfreq(1.0 / 256.0);

    std::cout << "FFT result:" << std::endl;
    for (size_t i = 0; i < fft_result.size(); i++)
    {
        std::cout << fft_freq(i) << ' ' << fft_result(i).real() << std::endl;
    }

    return 0;
}
