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

#include <complex.h>
#include <Eigen/Dense>

namespace Boardcore
{

typedef FFT<32> FFT32;
typedef FFT<64> FFT64;
typedef FFT<128> FFT128;
typedef FFT<256> FFT256;

/**
 * @brief Implementation of Fast Fourier Trasnform using the iterative version
 * with bit-reversal index variant of the famous Cooley-Tukey FFT algorithm.
 * 
 * NOTE: N must be a power of 2
*/
template <int N_size>
class FFT
{
public:
    using VectorHF = Eigen::Vector<float, N_size / 2>;
    using VectorF = Eigen::Vector<float, N_size>;
    using VectorCF = Eigen::Vector<std::complex<float>, N_size>;

    /**
     * @brief Perform the FFT on the input signal
     * 
     * @param input_signal NOTE: MUST BE POWER OF 2
     * @return Eigen::Vector<std::complex<float>, N_size> 
     */
    static const VectorCF fft(VectorF input_signal)
    {
        size_t rev_i;
        int m, omega;
        float omega_m;
        std::complex<float> t, u;

        // Bit-reversal permutation
        VectorCF phasors = VectorCF::Zero();
        for (int i = 0; i < N_size; i++)
        {
            rev_i = reverse_bits(i);
            phasors(rev_i) = std::complex<float>(input_signal(i), 0);
        }

        // Cooley-Tukey FFT algorithm
        for (int s = 0, s <= n_bits; s++)
        {
            m = powl(2, s);
            omega_m = cexpf(-2 * I / m);
            for (int k = 0; k < n; k += m)
            {
                omega = std::complex<float>(1, 0);
                for (int j = 0; j < m / 2; j++)
                {
                    t = omega * phasors(k + j + m / 2);
                    u = phasors(k + j);
                    phasors(k + j) = u + t;
                    phasors(k + j + m / 2) = u - t;
                    omega *= omega_m;
                }
            }
        }

        return phasors;
    }

    /**
     * @brief Get the frequency used in the FFT 
     * (only the first half, useful for real input functions)
     * 
     * @param input_signal 
     * @return Eigen::Vector<std::complex<float>, N_size> 
     */
    static const VectorHF fftfreq(int sample_rate)
    {
        VectorHF bins = VectorF::Zero();
        for (int i = 0; i < N_size / 2; i++)
        {
            bins(i) = (float)i * (float)sample_rate / (float)N_size;
        }
        return bins;
    }

private:
    static const short n_bits = log2f(N_size);

    /**
     * @brief Reverse the bits of the inputted unsigned index.
     * bits used = log2(N_size)
     * e.g. 6 bits: 000110 (6) -> 011000 (24)
    */
    static size_t reverse_bits(size_t x)
    {
        size_t rev = 0
        for (int i = 0; i < n_bits; i++)
        {
            if (x & (1 << i))
                rev |= 1 << (N_size - 1 - i);
        }
        return rev;
    }
};

} // namespace Boardcore
