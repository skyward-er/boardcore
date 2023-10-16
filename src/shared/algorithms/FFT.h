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
#include <math.h>

#include <Eigen/Dense>

namespace Boardcore
{

/**
 * @brief Implementation of Fast Fourier Transform using the iterative version
 * with bit-reversal index variant of the famous Cooley-Tukey FFT algorithm.
 *
 * NOTE: N must be a power of 2
 */
template <int N_size>
class FFT
{
public:
    using VectorF  = Eigen::Vector<float, N_size>;
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
        int m;
        float phi;
        std::complex<float> t, u, omega, omega_m;

        // Bit-reversal permutation
        VectorCF phasors = VectorCF::Zero();
        for (int i = 0; i < N_size; i++)
        {
            rev_i          = reverse_bits(i);
            phasors(rev_i) = std::complex<float>(input_signal(i), 0);
        }

        // Cooley-Tukey FFT algorithm
        for (int s = 1; s <= n_bits(N_size); s++)
        {
            m       = powl(2, s);
            phi     = -2 * EIGEN_PI / m;
            omega_m = std::complex<float>(cos(phi), sin(phi));
            for (int k = 0; k < N_size; k += m)
            {
                omega = std::complex<float>(1, 0);
                for (int j = 0; j < m / 2; j++)
                {
                    t = omega * phasors(k + j + m / 2);
                    u = phasors(k + j);

                    phasors(k + j)         = u + t;
                    phasors(k + j + m / 2) = u - t;

                    omega *= omega_m;
                }
            }
        }

        return phasors;
    }

    /**
     * @brief Get the frequency used in the FFT
     *
     * @param sample_rate in Hertz
     * @return Eigen::Vector<std::complex<float>, N_size>
     */
    static const VectorF fftfreq(float sample_rate)
    {
        VectorF bins = VectorF::Zero();
        for (int i = 0; i < N_size / 2; i++)
        {
            bins(i) = (float)i * sample_rate / (float)N_size;
        }
        for (int i = N_size / 2; i < N_size; i++)
        {
            bins(i) = (float)(i - N_size) * sample_rate / (float)N_size;
        }
        return bins;
    }

    /**
     * @brief Perform the inverse FFT on the input signal
     *
     * @param fft_vector
     * @param sample_rate in Hertz
     * @return Eigen::Vector<std::complex<float>, N_size>
     */
    static const VectorF ifft(VectorCF fft_vector, float sample_rate)
    {
        float amplitude, alpha, phi;
        VectorF ifft_vector = VectorF::Zero();
        VectorF bins        = fftfreq(sample_rate);

        for (int i = 0; i < N_size; i++)
        {
            for (int j = 0; j < N_size; j++)
            {
                // A = |f|
                amplitude = std::abs(fft_vector(j));
                // α = 2πf * t
                alpha = 2 * EIGEN_PI * bins(j) * (float)i / sample_rate;
                // ϕ = arg(f)
                phi = std::arg(fft_vector(j));

                // i = A * cos(α + ϕ)
                ifft_vector(i) += amplitude * cos(alpha + phi);
            }
            // scale down
            ifft_vector(i) /= N_size;
        }

        return ifft_vector;
    }

private:
    /**
     * @brief Get the number of bits to differentiate x elements
     */
    static short n_bits(size_t x) { return (short)log2(x); }

    /**
     * @brief Reverse the bits of the inputted unsigned index.
     * bits used = log2(N_size)
     * e.g. 6 bits: 000110 (6) -> 011000 (24)
     */
    static size_t reverse_bits(size_t x)
    {
        size_t rev = 0;
        short bits = n_bits(N_size);
        for (int i = 0; i < bits; i++)
        {
            if (x & (1 << i))
            {
                rev |= 1 << (bits - 1 - i);
            }
        }
        return rev;
    }
};

typedef FFT<8> FFT8;
typedef FFT<16> FFT16;
typedef FFT<32> FFT32;
typedef FFT<64> FFT64;
typedef FFT<128> FFT128;
typedef FFT<256> FFT256;

}  // namespace Boardcore
