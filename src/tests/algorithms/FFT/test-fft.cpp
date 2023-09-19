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

#include <algorithms/FFT.h>
#include <complex.h>
#include <math.h>

#include <Eigen/Core>

#include "test-fft-data.h"

using namespace Boardcore;
using namespace Eigen;

int main()
{
    bool failed = false;

    Eigen::Vector<float, SAMPLES> signal_vector =
        Eigen::Vector<float, SAMPLES>::Zero();
    for (int i = 0; i < SAMPLES; i++)
    {
        signal_vector(i) = SIGNAL[i];
    }

    Vector<std::complex<float>, SAMPLES> fft_result =
        FFT<SAMPLES>::fft(signal_vector);
    Vector<float, SAMPLES> fft_freq = FFT<SAMPLES>::fftfreq(SAMPLE_RATE);

    for (int i = 0; i < fft_result.size(); i++)
    {
        if (std::abs(1.0 / SAMPLES * std::abs(fft_result(i)) - INTENSITY[i]) >
            0.001)
        {
            printf("FFT result differs from the correct one [%d]: %f != %f\n",
                   i, fft_result(i).real(), INTENSITY[i]);
            failed = true;
        }
    }

    for (int i = 0; i < fft_freq.size(); i++)
    {
        if (fft_freq(i) != FREQUENCY[i])
        {
            printf("FFT freq differs from the correct one [%d]: %f != %f\n", i,
                   fft_freq(i), FREQUENCY[i]);
            failed = true;
        }
    }

    failed ? printf("FAILED\n") : printf("PASSED\n");

    return 0;
}
