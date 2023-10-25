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

namespace Boardcore
{

/**
 * @brief Online Low Pass filter with frequency-aware parameters
 */
class LowPass
{
public:
    struct LowPassConfig
    {
        float gain;
        float cutoffFreq;
        float lambda;
    };

    /**
     * @brief Construct an online Low Pass by providing each parameter
     *
     * @param gain The gain of the filter
     * @param cutoffFreq The cutoff frequency of the filter
     * @param lambda The lambda parameter of the filter
     *
     * @note WARNING: Initialize output at 0 at first
     * @note WARNING: frequency set by parameters, look for these in any case
     */
    LowPass(float gain, float cutoffFreq, float lambda);

    /**
     * @brief Construct an online Low Pass from a configuration
     *
     * @param config The configuration of the filter
     *
     * @note WARNING: Initialize output at 0 at first
     * @note WARNING: frequency set by parameters, look for these in any case
     */
    explicit LowPass(const LowPassConfig& config);

    /**
     * @brief Filter the input
     *
     * @param input The input to filter
     */
    float filter(float input);

private:
    float gain, cutoffFreq, lambda, output;
};

}  // namespace Boardcore
