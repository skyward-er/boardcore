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

// ======= Sensor Fault Detection Model (SFDAscent) =======

#include "SFDAscent.h"

#include <algorithms/FFT.h>

namespace Boardcore
{

SFDAscent::SFDAscent(const SFDAscentConfig& config)
    : svm(config.modelParameters)
{
}

SFDAscent::FeaturesVec SFDAscent::getFeatures(const SFDVectorIn& input)
{
    float delta, min, max, u, var, s2, m4, rfmean, rfvar;
    SFDVectorIn rfourier, x0;
    SFDVectorIn data     = SFDVectorIn::Zero();
    FeaturesVec features = FeaturesVec::Zero();

    min   = input.minCoeff();
    max   = input.maxCoeff();
    delta = max - min;
    for (int i = 0; i < LEN_CHUNK; i++)
        data(i) = (input(i) - min) / (std::max(delta, 1e-25f) * 2) - 1;
    u   = data.mean();
    x0  = data - u * SFDVectorIn::Ones();
    var = x0.squaredNorm() / LEN_CHUNK;
    s2  = x0.array().pow(2).mean();
    m4  = x0.array().pow(4).mean();

    rfourier = FFT32::fft(data).real();  // TODO: fix complex -> float
    rfmean   = rfourier.mean();
    rfvar = (rfourier - rfmean * SFDVectorIn::Ones()).squaredNorm() / LEN_CHUNK;

    features(0) = delta;
    features(1) = var;
    features(2) = m4 / std::pow(s2, 2);
    features(3) = data.array().pow(5).mean();
    features(4) = rfvar;
    features(5) = rfourier.cwiseAbs().sum();

    return features;
}

bool SFDAscent::classify(const SFDVectorIn& input)
{
    FeaturesVec features = getFeatures(input);
    float score          = svm.score(features);

    return score < 0;
}

}  // namespace Boardcore
